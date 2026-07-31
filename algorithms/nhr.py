"""
Nonlinear Metropolis-Adjusted Hit-and-Run (NHR), ported from the reference
C++ implementation (rai's src/Optim/NLP_Sampler.cpp, commit 8547d438,
accompanying Toussaint/Braun/Ortiz-Haro's "NLP Sampling" paper) rather than
the paper's own simplified pseudocode. The two differ in one important way:
the reference's hit-and-run step uses a lenient "doesn't get worse" accept
criterion, paired with a separate, conditional Gauss-Newton cleanup step -
not the paper's strict-feasibility-per-step design.

Equality constraints are handled throughout via tangent-space direction
projection plus a walk-only epsilon-margin tube, matching the reference.
"""

import json
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable, Optional, Dict, Any, Literal, List

import numpy as np


Array = np.ndarray


def _json_safe(value: Any) -> Any:
    """Convert NumPy-heavy objects into JSON-serializable Python values."""
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, np.bool_):
        return bool(value)
    if isinstance(value, dict):
        return {str(key): _json_safe(val) for key, val in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, Path):
        return str(value)
    return value


# -----------------------------------------------------------------------------
# Options
# -----------------------------------------------------------------------------

@dataclass
class NHROptions:
    """
    Ports NLP_Sampler_Options (src/Optim/NLP_Sampler.h), with names adapted
    to this project's style. Field comments note which reference field
    each one ports.
    """
    num_samples: int = 1000
    burn_in: int = 100
    thinning: int = 1

    delta_max: Optional[float] = None
    # step_hit_and_run's box-clip half-width is 2*delta_max, matching the
    # reference's LineSampler(2.*opt.slackMaxStep). Left as None, it
    # defaults to slack_max_step below (the reference's actual, conflated
    # behaviour); set it explicitly to decouple the two roles.

    walk_margin: float = 0.1           # ports hitRunEqMargin
    eps_slack_increase: float = 0.05   # ports opt.eps

    penalty_mu: float = 1.0            # ports opt.penaltyMu
    slack_step_alpha: float = 1.0      # ports opt.slackStepAlpha
    slack_max_step: float = 0.1        # ports opt.slackMaxStep
    slack_reg_lambda: float = 1e-2     # ports opt.slackRegLambda

    good_err_tol: float = 0.01         # ports the hardcoded 0.01 in run_downhill/run_interior
    downhill_max_steps: int = 50       # ports opt.downhillMaxSteps
    hit_and_run_inner_tries: int = 10  # ports the hardcoded 10 in step_hit_and_run

    interior_method: Literal["HR", "mRRT"] = "HR"
    # Which interior-sampling method run_interior uses. "HR" is
    # step_hit_and_run (Nonlinear Metropolis-Adjusted Hit-and-Run), the
    # default and the only method this project used before. "mRRT" is
    # Manifold-RRT (Algorithm 4): instead of a constraint-respecting line
    # search from the current point, it jumps to the nearest previously
    # visited point and takes one fixed-size step from there toward a
    # random target, tangent-projected at that anchor - inequalities are
    # only handled afterward via the same conditional cleanup HR uses, so
    # a single very tight inequality can't collapse the whole step the
    # way it can for HR's shared-beta line search. Per the paper's own
    # results, mRRT tends to outperform HR specifically on problems
    # dominated by hard equality constraints (the robotics category) -
    # this project's grasp problem is exactly that shape.
    interior_noise_sigma: float = 0.1  # ports opt.interiorNoiseSigma - mRRT's fixed step size

    constraint_tol: Optional[float] = None
    # Final per-row tolerance for is_feasible()/manifold-projection
    # filtering - NOT the walk's internal margin (that's walk_margin).
    # Left as None, defaults to good_err_tol: run_interior only ever
    # certifies a recorded sample's *combined* slack sum is <=
    # good_err_tol, so a per-row gate much tighter than that (e.g. a
    # fixed 1e-8) would reject almost every sample no cleanup step was
    # ever asked to satisfy - g in particular is never separately
    # polished by project_samples_to_manifold (only h is, via
    # slack_reduce_equalities). Set explicitly for a stricter/looser
    # final filter independent of the walk's own internal tolerance.

    finite_difference_step: float = 1e-6
    random_seed: int = 0

    verbose: bool = True

    def __post_init__(self):
        if self.delta_max is None:
            self.delta_max = self.slack_max_step
        if self.constraint_tol is None:
            self.constraint_tol = self.good_err_tol


RestartStrategy = Literal["uniform", "distance", "direction"]


@dataclass
class RestartOptions:
    num_restarts: int = 10
    strategy: RestartStrategy = "uniform"
    candidates_per_restart: int = 50
    random_seed: int = 0
    keep_failed_phase1_seeds: bool = False


# -----------------------------------------------------------------------------
# Pure math primitives
# -----------------------------------------------------------------------------

def finite_difference_jacobian(
    fun: Callable[[Array], Array],
    x: Array,
    step: float = 1e-6,
) -> Array:
    """
    Central finite-difference Jacobian.

    Use this only for prototyping.
    Later replace with analytic/autodiff Jacobians if possible.
    """
    x = np.asarray(x, dtype=float)
    y0 = np.asarray(fun(x), dtype=float)

    J = np.zeros((len(y0), len(x)))

    for j in range(len(x)):
        xp = x.copy()
        xm = x.copy()
        xp[j] += step
        xm[j] -= step

        J[:, j] = (fun(xp) - fun(xm)) / (2.0 * step)

    return J


def clip_interval_with_linear_ineq(
    beta_lo: float,
    beta_up: float,
    g_bar: Array,
    a: Array,
    tol: float = 1e-12,
) -> tuple[float, float]:
    """
    Clip beta interval using linear inequalities:

        g_bar + beta * a <= 0

    This is the core hit-and-run clipping operation (ports LineSampler::clip_beta).
    """
    g_bar = np.asarray(g_bar, dtype=float).reshape(-1)
    a = np.asarray(a, dtype=float).reshape(-1)

    for gi, ai in zip(g_bar, a):
        if abs(ai) < tol:
            if gi > 0:
                return 1.0, 0.0
            continue

        beta_boundary = -gi / ai

        if ai > 0:
            beta_up = min(beta_up, beta_boundary)
        else:
            beta_lo = max(beta_lo, beta_boundary)

        if beta_lo > beta_up:
            return beta_lo, beta_up

    return beta_lo, beta_up


def clip_interval_with_box_bounds(
    x: Array,
    d: Array,
    lower: Array,
    upper: Array,
    beta_lo: float,
    beta_up: float,
) -> tuple[float, float]:
    """
    Clip beta interval so that lower <= x + beta*d <= upper, i.e. two
    linear inequalities (lower-x)+beta(-d)<=0 and (x-upper)+beta(d)<=0.
    """
    x = np.asarray(x, dtype=float)
    d = np.asarray(d, dtype=float)

    beta_lo, beta_up = clip_interval_with_linear_ineq(beta_lo, beta_up, g_bar=lower - x, a=-d)
    beta_lo, beta_up = clip_interval_with_linear_ineq(beta_lo, beta_up, g_bar=x - upper, a=d)

    return beta_lo, beta_up


def project_direction_to_equality_tangent(
    d: Array,
    Jh_x: Array,
    regularization: float = 1e-8,
) -> Array:
    """
    Project a direction into the tangent space of h(x)=0.

    Tangent projection:

        d_tan = (I - P_h) d
        P_h = J_h^T (J_h J_h^T + reg I)^(-1) J_h

    Algebraically equivalent to the reference's pinv(Jh^T Jh) formulation
    (get_rnd_direction), just inverting the smaller m x m Gram matrix
    (m = number of equality rows) instead of an n x n pseudo-inverse -
    more efficient when m << n, and already regularized for rank-deficiency.
    """
    d = np.asarray(d, dtype=float).reshape(-1)
    Jh_x = np.asarray(Jh_x, dtype=float)

    if Jh_x.size == 0:
        return d

    if Jh_x.ndim == 1:
        Jh_x = Jh_x.reshape(1, -1)

    m = Jh_x.shape[0]
    A = Jh_x @ Jh_x.T + regularization * np.eye(m)
    normal_component = Jh_x.T @ np.linalg.solve(A, Jh_x @ d)

    return d - normal_component


# -----------------------------------------------------------------------------
# Evaluation model - ports NLP_Walker::Eval
# -----------------------------------------------------------------------------

@dataclass
class Eval:
    """
    Ports NLP_Walker::Eval (src/Optim/NLP_Sampler.h) as a plain data
    container - no methods/side effects, and no phi/featureTypes
    indirection (unnecessary here: g/h are already separately-typed
    callables in this project, unlike the reference's generic NLP).
    """
    x: Array
    g: Array     # inequality features, shape (m_g,)
    Jg: Array    # shape (m_g, n)
    h: Array     # equality features, shape (m_h,) - empty array if no equalities
    Jh: Array    # shape (m_h, n)
    s: Array     # slack: ReLU(g) concatenated with |h| (sign-corrected), shape (m_g+m_h,)
    Js: Array    # Jacobian of s, rows zeroed/sign-flipped to match
    gpos: Array  # ReLU(g) only, captured before h is appended (ports ev.gpos)
    err: float   # sum(s) - the aggregate infeasibility scalar


def evaluate_point(
    x: Array,
    g: Callable[[Array], Array],
    Jg: Callable[[Array], Array],
    h: Optional[Callable[[Array], Array]] = None,
    Jh: Optional[Callable[[Array], Array]] = None,
) -> Eval:
    """
    Ports Eval::eval(). Pure function: computes the slack vector
    s = (ReLU(g), |h|) and its Jacobian, plus the aggregate infeasibility
    scalar err = sum(s).
    """
    x = np.asarray(x, dtype=float)
    n = len(x)

    gx = np.asarray(g(x), dtype=float).reshape(-1)
    Jgx = np.asarray(Jg(x), dtype=float).reshape(len(gx), n)

    if h is not None:
        hx = np.asarray(h(x), dtype=float).reshape(-1)
        Jhx = np.asarray(Jh(x), dtype=float).reshape(len(hx), n)
    else:
        hx = np.zeros(0)
        Jhx = np.zeros((0, n))

    s = np.where(gx > 0.0, gx, 0.0)
    Js = Jgx.copy()
    Js[gx <= 0.0, :] = 0.0
    gpos = s.copy()

    s = np.concatenate([s, hx])
    Js = np.vstack([Js, Jhx])
    neg = np.zeros(len(s), dtype=bool)
    neg[len(gx):] = hx < 0.0
    s[neg] *= -1.0
    Js[neg, :] *= -1.0

    return Eval(x=x, g=gx, Jg=Jgx, h=hx, Jh=Jhx, s=s, Js=Js, gpos=gpos, err=float(np.sum(s)))


def apply_equality_margin(ev: Eval, margin: float) -> tuple[Array, Array]:
    """
    Ports Eval::convert_eq_to_ineq(margin). Returns (g_aug, Jg_aug), the
    inequality set augmented with h(x) turned into a +-margin tube:

        h(x) - margin <= 0
       -h(x) - margin <= 0

    Deliberately non-mutating (the reference mutates ev.g/Jg in place) so
    callers never have to reason about what ev.g means before vs. after
    this call.
    """
    if ev.h.size == 0 or margin <= 0.0:
        return ev.g, ev.Jg
    g_aug = np.concatenate([ev.g, ev.h - margin, -ev.h - margin])
    Jg_aug = np.vstack([ev.Jg, ev.Jh, -ev.Jh])
    return g_aug, Jg_aug


def sample_random_tangent_direction(
    x: Array,
    Jh: Optional[Callable[[Array], Array]],
    rng: np.random.Generator,
    regularization: float = 1e-8,
) -> Array:
    """
    Ports get_rnd_direction(). Draws d ~ N(0,I); if Jh is given, projects d
    into the tangent space of h(x)=0. Always returns a unit vector.
    """
    d = rng.normal(size=len(x))

    if Jh is not None:
        Jh_x = np.asarray(Jh(x), dtype=float)
        if Jh_x.size:
            d = project_direction_to_equality_tangent(d, Jh_x, regularization)

    d_norm = np.linalg.norm(d)
    if d_norm < 1e-12:
        raise ValueError(
            "Random direction collapsed to zero after tangent projection - "
            "the equality constraints leave no free directions at this x."
        )
    return d / d_norm


def gauss_newton_slack_delta(s: Array, Js: Array, n: int, penalty_mu: float, lam: float) -> Array:
    """
    Pure helper: the numerator of step_GaussNewton's slack-mode direction.

        Hinv = inv(2*penalty_mu*Js^T@Js + lam*I)
        delta = -2*penalty_mu*Hinv@Js^T@s
    """
    if s.size == 0:
        return np.zeros(n)
    H = 2.0 * penalty_mu * (Js.T @ Js) + lam * np.eye(n)
    Hinv = np.linalg.inv(H)
    return -2.0 * penalty_mu * (Hinv @ (Js.T @ s))


def apply_trust_region(delta: Array, alpha: float, max_step: float) -> Array:
    """
    Pure helper, ports step_GaussNewton's step-size adaptation: scale by
    alpha, then if the result exceeds max_step, rescale to norm exactly
    max_step (a single rescale, not iterative shrinking).
    """
    delta = delta * alpha
    length = np.linalg.norm(delta)
    if length > max_step and length > 0:
        delta = delta * (max_step / length)
    return delta


# -----------------------------------------------------------------------------
# Walker state and stepping - ports NLP_Walker
# -----------------------------------------------------------------------------

@dataclass
class ProblemFns:
    """Bundles the problem definition so it isn't threaded as many separate
    positional args through every step_* function below."""
    g: Callable[[Array], Array]
    Jg: Callable[[Array], Array]
    lower: Array
    upper: Array
    h: Optional[Callable[[Array], Array]] = None
    Jh: Optional[Callable[[Array], Array]] = None
    f: Optional[Callable[[Array], float]] = None
    extra_check: Optional[Callable[[Array], bool]] = None


@dataclass
class WalkerState:
    """Ports NLP_Walker's mutable (x, ev, ev_stored, evals) state."""
    x: Array
    ev: Eval
    ev_stored: Optional[Eval] = None
    evals: int = 0

    @classmethod
    def initialize(cls, x0: Array, problem: ProblemFns) -> "WalkerState":
        x0 = np.asarray(x0, dtype=float).copy()
        ev = evaluate_point(x0, problem.g, problem.Jg, problem.h, problem.Jh)
        return cls(x=x0, ev=ev, ev_stored=None, evals=1)


def ensure_eval(state: WalkerState, problem: ProblemFns) -> Eval:
    """
    Ports ensure_eval()/Eval::eval's memoization: if state.ev is already at
    (within 1e-10 of) state.x, returns it unchanged; otherwise recomputes.
    Drake FK is the dominant cost this project cares about, so this cache
    is load-bearing, not an afterthought.
    """
    if state.ev is not None and np.max(np.abs(state.ev.x - state.x)) < 1e-10:
        return state.ev
    state.ev = evaluate_point(state.x, problem.g, problem.Jg, problem.h, problem.Jh)
    state.evals += 1
    return state.ev


def store_eval(state: WalkerState, problem: ProblemFns) -> None:
    """state.ev_stored = ensure_eval(state, problem) (a snapshot)."""
    state.ev_stored = ensure_eval(state, problem)


def bound_clip(state: WalkerState, problem: ProblemFns) -> None:
    """Ports bound_clip(): plain per-coordinate clamp of x into [lower, upper]."""
    state.x = np.clip(state.x, problem.lower, problem.upper)


def step_gauss_newton(
    state: WalkerState,
    problem: ProblemFns,
    slack_mode: bool = True,
    penalty_mu: float = 1.0,
    alpha: Optional[float] = None,
    max_step: Optional[float] = None,
    lam: float = 1e-2,
    options: Optional[NHROptions] = None,
) -> None:
    """
    Ports NLP_Walker::step_GaussNewton (slack-mode only - this project has
    no sos/energy residual to port the other branch for). A single damped
    Gauss-Newton step toward zero slack, capped by a trust region. Always
    "succeeds" in the sense the reference does - no internal accept/reject,
    the caller decides what to do with the result.
    """
    ensure_eval(state, problem)
    ev = state.ev

    if alpha is None:
        alpha = options.slack_step_alpha if options is not None else 1.0
    if max_step is None:
        max_step = options.slack_max_step if options is not None else 0.1

    delta = gauss_newton_slack_delta(ev.s, ev.Js, len(state.x), penalty_mu, lam)
    delta = apply_trust_region(delta, alpha, max_step)

    state.x = state.x + delta  # new array - never mutate state.x in place (see WalkerState note below)
    ensure_eval(state, problem)


def step_hit_and_run(
    state: WalkerState,
    problem: ProblemFns,
    options: NHROptions,
    rng: np.random.Generator,
) -> Dict[str, Any]:
    """
    Ports NLP_Walker::step_hit_and_run() - the core sampling step:

    1. Sample a direction, tangent to h(x)=0 if h is given.
    2. Clip [beta_lo, beta_up] using box bounds, then repeatedly using the
       current point's linearised inequalities (equalities folded in as a
       +-walk_margin tube, looser than the tolerance used to judge a final
       sample).
    3. Each trial moves x by a sampled beta *from wherever the previous
       trial left it* - a chained walk, not independent retries from a
       fixed anchor - and accepts as soon as the worst inequality
       violation is no worse than at the start and total slack hasn't
       grown by more than eps_slack_increase: a lenient "doesn't get
       worse" criterion, not strict feasibility.
    4. If all tries are exhausted without accepting, reverts to the exact
       state before this call.

    Returns a diagnostics dict; state.x/state.ev are updated in place.
    """
    store_eval(state, problem)
    ev = state.ev
    ev_stored = state.ev_stored

    g_aug, Jg_aug = apply_equality_margin(ev, options.walk_margin)
    g0 = max(np.max(g_aug), 0.0) if g_aug.size else 0.0
    s0 = ev.s  # raw slack at the start - NOT re-derived after the margin conversion

    direction = sample_random_tangent_direction(state.x, problem.Jh, rng)

    beta_lo, beta_up = -2.0 * options.delta_max, 2.0 * options.delta_max
    beta_lo, beta_up = clip_interval_with_box_bounds(
        x=state.x, d=direction, lower=problem.lower, upper=problem.upper,
        beta_lo=beta_lo, beta_up=beta_up,
    )

    reason = "exhausted_tries"
    for inner_try in range(options.hit_and_run_inner_tries):
        # ev.x == state.x always holds here (ensure_eval() below keeps them
        # in sync every iteration), so this linearisation-recentring term
        # is always ~0 - kept anyway to mirror the reference's actual
        # behaviour exactly, not a "simplified" reinterpretation of it.
        beta_lo, beta_up = clip_interval_with_linear_ineq(
            beta_lo=beta_lo, beta_up=beta_up,
            g_bar=g_aug + Jg_aug @ (state.x - ev.x),
            a=Jg_aug @ direction,
        )
        if beta_lo >= beta_up:
            reason = "empty_interval"
            break

        beta = rng.uniform(beta_lo, beta_up)
        state.x = state.x + beta * direction  # chained: not reset between tries
        ev = ensure_eval(state, problem)
        g_aug, Jg_aug = apply_equality_margin(ev, options.walk_margin)

        ineq_ok = (g_aug.size == 0) or (np.max(g_aug) <= g0)
        slack_ok = np.sum(ev.s) <= np.sum(s0) + options.eps_slack_increase

        if ineq_ok and slack_ok:
            if problem.f is not None:
                f_new = float(problem.f(state.x))
                f_old = float(problem.f(ev_stored.x))
                if f_new < f_old or np.log(rng.uniform()) < (f_old - f_new):
                    return {"accepted": True, "reason": "mh_accept", "inner_try": inner_try}
                state.x, state.ev = ev_stored.x, ev_stored
                return {"accepted": False, "reason": "mh_reject", "inner_try": inner_try}
            return {"accepted": True, "reason": "accepted", "inner_try": inner_try}

    state.x, state.ev = ev_stored.x, ev_stored
    return {"accepted": False, "reason": reason, "inner_try": options.hit_and_run_inner_tries}


def run_downhill(state: WalkerState, problem: ProblemFns, options: NHROptions) -> bool:
    """
    Ports NLP_Walker::run_downhill(): a damped Gauss-Newton descent toward
    feasibility, with a Wolfe-style step-size adaptation (shrink alpha and
    revert on a slack increase; otherwise grow alpha, capped at 1) - without
    this adaptation it's just a fixed-step GN loop with no safety net.
    """
    ensure_eval(state, problem)
    alpha = options.slack_step_alpha

    for _ in range(options.downhill_max_steps):
        store_eval(state, problem)
        step_gauss_newton(
            state, problem, slack_mode=True, penalty_mu=options.penalty_mu,
            alpha=alpha, max_step=options.slack_max_step, lam=options.slack_reg_lambda,
        )
        bound_clip(state, problem)
        # No ensure_eval() here - a faithful quirk of the reference: the
        # Wolfe check below can transiently use the pre-clip evaluation,
        # self-healing on the next iteration's store_eval() call.

        if state.ev.err > state.ev_stored.err:
            alpha *= 0.5
            state.x, state.ev = state.ev_stored.x, state.ev_stored
        else:
            alpha = min(1.0, alpha * 1.2)

        if state.ev.err <= options.good_err_tol:
            return True

    return False


@dataclass
class ManifoldTree:
    """
    Minimal growing nearest-neighbor store for mRRT: parallel lists of
    visited anchor points and their local equality-tangent projection
    matrices. Ports the ANN + annPh bookkeeping in run_interior's
    manifoldRRT branch. Brute-force nearest-neighbor is fine at the
    sample counts this project uses (hundreds to low thousands per
    restart).
    """
    points: List[Array]
    projectors: List[Array]

    @classmethod
    def empty(cls) -> "ManifoldTree":
        return cls(points=[], projectors=[])

    def append(self, x: Array, Jh_x: Optional[Array], regularization: float = 1e-6) -> None:
        n = len(x)
        if Jh_x is not None and np.asarray(Jh_x).size:
            Jh_x = np.atleast_2d(np.asarray(Jh_x, dtype=float))
            m = Jh_x.shape[0]
            A = Jh_x @ Jh_x.T + regularization * np.eye(m)
            P = np.eye(n) - Jh_x.T @ np.linalg.solve(A, Jh_x)
        else:
            P = np.eye(n)
        self.points.append(np.asarray(x, dtype=float).copy())
        self.projectors.append(P)

    def nearest(self, x_target: Array) -> tuple[Array, Array]:
        pts = np.asarray(self.points)
        d2 = np.sum((pts - x_target) ** 2, axis=1)
        p = int(np.argmin(d2))
        return self.points[p], self.projectors[p]


def mrrt_step(
    state: WalkerState,
    problem: ProblemFns,
    options: NHROptions,
    rng: np.random.Generator,
    tree: ManifoldTree,
) -> Dict[str, Any]:
    """
    Ports run_interior's manifoldRRT branch: sample a uniform random
    target in the box, find the nearest already-visited anchor, and take
    one fixed-size (interior_noise_sigma) step from that anchor toward
    the target, projected tangent to h(x)=0 as evaluated at the anchor
    (not re-evaluated at the walker's current position). Inequalities are
    not consulted at all when building this step - only run_interior's
    shared conditional Gauss-Newton cleanup handles them, afterward.
    """
    x_target = sample_uniform_box(problem.lower, problem.upper, rng)
    anchor, Ph = tree.nearest(x_target)

    direction = Ph @ (x_target - anchor)
    length = np.linalg.norm(direction)
    if length < 1e-12:
        state.x = anchor.copy()
        ensure_eval(state, problem)
        return {"accepted": True, "reason": "mrrt_zero_length"}

    state.x = anchor + direction * (options.interior_noise_sigma / length)
    ensure_eval(state, problem)
    return {"accepted": True, "reason": "mrrt_step"}


def run_interior(
    state: WalkerState, problem: ProblemFns, options: NHROptions, rng: np.random.Generator,
) -> tuple[Array, list[Dict[str, Any]]]:
    """
    Ports NLP_Walker::run_interior(): the two-phase-per-iteration interior
    sampling loop. Each iteration takes one lenient hit-and-run step, then
    - only if that step's result isn't already "good" - applies one
    conditional Gauss-Newton cleanup step. A point is only ever recorded
    as a sample if it's good (and past burn-in), independent of whether
    the hit-and-run step itself was "accepted".
    """
    burn_in = max(options.burn_in, 0)
    sample_steps = max(options.num_samples, 1)
    interior_steps = burn_in + sample_steps - 1

    samples: List[Array] = []
    diagnostics: List[Dict[str, Any]] = []

    use_mrrt = options.interior_method == "mRRT"
    tree = ManifoldTree.empty() if use_mrrt else None

    t = 0
    while True:
        ensure_eval(state, problem)
        good = state.ev.err <= options.good_err_tol
        if good and problem.extra_check is not None:
            good = good and bool(problem.extra_check(state.x))

        if use_mrrt:
            Jh_x = problem.Jh(state.x) if problem.Jh is not None else None
            tree.append(state.x, Jh_x)

        if good and t >= burn_in:
            samples.append(state.x.copy())

        if t >= interior_steps:
            diagnostics.append({"step": t, "is_burn_in": t < burn_in, "err": state.ev.err, "good": good})
            break

        if use_mrrt:
            step_info = mrrt_step(state, problem, options, rng, tree)
        else:
            step_info = step_hit_and_run(state, problem, options, rng)

        ensure_eval(state, problem)
        good_pre_cleanup = state.ev.err <= options.good_err_tol
        cleanup_ran = False
        if options.slack_step_alpha > 0 and not good_pre_cleanup:
            step_gauss_newton(
                state, problem, slack_mode=True, penalty_mu=options.penalty_mu,
                alpha=1.0, max_step=options.slack_max_step, lam=options.slack_reg_lambda,
            )
            bound_clip(state, problem)
            cleanup_ran = True

        ensure_eval(state, problem)  # present here (unlike run_downhill), matching the reference

        diagnostics.append({
            "step": t, "is_burn_in": t < burn_in,
            "hr_accepted": step_info["accepted"], "hr_reason": step_info["reason"],
            "cleanup_ran": cleanup_ran, "err": state.ev.err,
        })

        if options.verbose and t % max(1, (interior_steps + 1) // 10) == 0:
            print(f"step {t:05d}/{interior_steps} | samples {len(samples):05d} | "
                  f"err {state.ev.err:.3e} | last: {step_info['reason']}")

        t += 1

    return np.asarray(samples), diagnostics


def _default_jacobian(
    fn: Optional[Callable[[Array], Array]],
    value_fn: Optional[Callable[[Array], Array]],
    step: float,
) -> Optional[Callable[[Array], Array]]:
    """If fn is None and value_fn is given, fall back to a finite-difference
    Jacobian of value_fn. Otherwise returns fn unchanged."""
    if fn is not None or value_fn is None:
        return fn
    return lambda z: finite_difference_jacobian(value_fn, z, step=step)


def run_downhill_phase1(
    seed: Array,
    g: Callable[[Array], Array],
    lower: Array,
    upper: Array,
    options: NHROptions,
    Jg: Optional[Callable[[Array], Array]] = None,
    h: Optional[Callable[[Array], Array]] = None,
    Jh: Optional[Callable[[Array], Array]] = None,
) -> Optional[Array]:
    """
    Native Phase 1: ports run_downhill() as a phase1(seed)->Optional[Array]
    callable, matching the shape restarting_nhr_sample already expects. No
    IK/Drake-specific dependency - only needs g/Jg/h/Jh.
    """
    Jg_eval = _default_jacobian(Jg, g, options.finite_difference_step)
    Jh_eval = _default_jacobian(Jh, h, options.finite_difference_step)

    problem = ProblemFns(g=g, Jg=Jg_eval, lower=lower, upper=upper, h=h, Jh=Jh_eval)
    seed = np.clip(np.asarray(seed, dtype=float), lower, upper)
    state = WalkerState.initialize(seed, problem)

    if run_downhill(state, problem, options):
        return state.x.copy()
    return None


# -----------------------------------------------------------------------------
# Public sampling entry points
# -----------------------------------------------------------------------------

def is_feasible(
    x: Array,
    g: Callable[[Array], Array],
    constraint_tol: float = 1e-8,
    h: Optional[Callable[[Array], Array]] = None,
    equality_tol: float = 1e-4,
    extra_check: Optional[Callable[[Array], bool]] = None,
) -> bool:
    """
    Feasible means g(x) <= constraint_tol, and (if h is given)
    |h(x)| <= equality_tol, plus optional extra boolean checks.
    """
    gx = np.asarray(g(x), dtype=float)
    if np.any(gx > constraint_tol):
        return False

    if h is not None:
        hx = np.asarray(h(x), dtype=float)
        if np.any(np.abs(hx) > equality_tol):
            return False

    if extra_check is not None and not extra_check(x):
        return False

    return True


def nhr_sample(
    x0: Array,
    g: Callable[[Array], Array],
    lower: Array,
    upper: Array,
    Jg: Optional[Callable[[Array], Array]] = None,
    f: Optional[Callable[[Array], float]] = None,
    extra_check: Optional[Callable[[Array], bool]] = None,
    options: Optional[NHROptions] = None,
    h: Optional[Callable[[Array], Array]] = None,
    Jh: Optional[Callable[[Array], Array]] = None,
    equality_tol: float = 1e-4,
    project_samples_to_manifold: bool = False,
    projection_iters: int = 10,
) -> tuple[Array, list[Dict[str, Any]]]:
    """
    Run Nonlinear Metropolis-Adjusted Hit-and-Run, following the reference
    implementation's actual two-phase-per-iteration interior sampling loop
    (run_interior): a lenient hit-and-run step, a conditional Gauss-Newton
    cleanup, and recording only points that are actually good.

    x0 must already be near-feasible (run Phase 1 first - e.g.
    run_downhill_phase1 for a native, Drake-free downhill Phase 1).

    Parameters
    ----------
    x0: initial near-feasible point.
    g, Jg: inequalities g(x)<=0 and their Jacobian (finite-difference if Jg is None).
    lower, upper: box bounds on x.
    f: optional scalar energy; if None, uniform sampling over the feasible set.
    extra_check: optional non-differentiable boolean checker; gates only
        whether a point that's already "good" gets recorded as a sample.
    options: NHROptions.
    h, Jh: optional equality constraints h(x)=0 and their Jacobian.
    equality_tol: tolerance used only for the optional post-hoc
        project_samples_to_manifold filtering (via is_feasible) - NOT the
        walk's own margin (options.walk_margin).
    project_samples_to_manifold: if True, project each recorded sample
        back toward h(x)=0 with slack_reduce_equalities, keeping it only
        if it's still feasible afterward.
    projection_iters: Gauss-Newton iterations used for that projection.

    Returns
    -------
    samples: array of recorded samples, shape (n_samples, dim).
    diagnostics: list of dicts describing each interior-sampling iteration.
    """
    if options is None:
        options = NHROptions()

    Jg_eval = _default_jacobian(Jg, g, options.finite_difference_step)
    Jh_eval = _default_jacobian(Jh, h, options.finite_difference_step)

    x0 = np.asarray(x0, dtype=float).copy()
    lower = np.asarray(lower, dtype=float)
    upper = np.asarray(upper, dtype=float)

    if x0.shape != lower.shape or x0.shape != upper.shape:
        raise ValueError("x0, lower, and upper must have the same shape.")
    if not np.all(lower <= upper):
        raise ValueError("All lower bounds must be <= upper bounds.")
    if not np.all((x0 >= lower) & (x0 <= upper)):
        raise ValueError("x0 must be inside the box bounds.")

    problem = ProblemFns(g=g, Jg=Jg_eval, lower=lower, upper=upper, h=h, Jh=Jh_eval, f=f, extra_check=extra_check)
    state = WalkerState.initialize(x0, problem)

    if state.ev.err > options.good_err_tol:
        raise ValueError(
            f"x0 is not feasible enough (err={state.ev.err:.3g} > "
            f"good_err_tol={options.good_err_tol}). Run Phase 1 first."
        )

    rng = np.random.default_rng(options.random_seed)
    samples, diagnostics = run_interior(state, problem, options, rng)

    if options.thinning > 1 and len(samples):
        samples = samples[::options.thinning]

    if project_samples_to_manifold and len(samples) > 0:
        projected = []
        for sample in samples:
            polished = sample
            if h is not None:
                polished = slack_reduce_equalities(
                    x=polished, h=h, Jh=Jh_eval, lower=lower, upper=upper,
                    finite_difference_step=options.finite_difference_step,
                    max_iters=projection_iters, tol=options.constraint_tol,
                )

            # slack_reduce_equalities only targets h. run_interior's own
            # cleanup only ever certified a recorded sample's *combined*
            # slack sum was <= good_err_tol, not each row's own tolerance -
            # so g can still be meaningfully violated here. Polish the full
            # slack (g and h together) with the same conditional
            # Gauss-Newton step run_interior's cleanup uses, so the final
            # is_feasible check below has something it was actually asked
            # to satisfy. Always spend the full iteration budget rather
            # than early-exiting on options.constraint_tol - that's the
            # same tolerance is_feasible checks against below, and (with
            # its default now tied to good_err_tol, see NHROptions) every
            # recorded sample already satisfies it on entry, which would
            # make this loop a no-op right when it's needed most. A few
            # extra GN steps past convergence are cheap: the step size
            # itself shrinks to ~0 once the slack does.
            polish_state = WalkerState.initialize(polished, problem)
            for _ in range(projection_iters):
                step_gauss_newton(
                    polish_state, problem, slack_mode=True, penalty_mu=options.penalty_mu,
                    alpha=1.0, max_step=options.slack_max_step, lam=options.slack_reg_lambda,
                )
                bound_clip(polish_state, problem)
                ensure_eval(polish_state, problem)
            polished = polish_state.x

            if is_feasible(polished, g, options.constraint_tol, h=h,
                            equality_tol=equality_tol, extra_check=extra_check):
                projected.append(polished)
        samples = np.asarray(projected)

    return samples, diagnostics


def nhr_sample_with_equalities(
    x0: Array,
    g: Callable[[Array], Array],
    h: Callable[[Array], Array],
    lower: Array,
    upper: Array,
    Jg: Optional[Callable[[Array], Array]] = None,
    Jh: Optional[Callable[[Array], Array]] = None,
    f: Optional[Callable[[Array], float]] = None,
    extra_check: Optional[Callable[[Array], bool]] = None,
    options: Optional[NHROptions] = None,
    equality_tol: float = 1e-4,
    project_samples_to_manifold: bool = False,
    projection_iters: int = 10,
) -> tuple[Array, list[Dict[str, Any]]]:
    """Backward-compatible wrapper around nhr_sample's equality-aware path."""
    return nhr_sample(
        x0=x0, g=g, lower=lower, upper=upper, Jg=Jg, f=f, extra_check=extra_check,
        options=options, h=h, Jh=Jh, equality_tol=equality_tol,
        project_samples_to_manifold=project_samples_to_manifold, projection_iters=projection_iters,
    )


# -----------------------------------------------------------------------------
# Restart seeding
# -----------------------------------------------------------------------------

def sample_uniform_box(lower: Array, upper: Array, rng: np.random.Generator) -> Array:
    lower = np.asarray(lower, dtype=float)
    upper = np.asarray(upper, dtype=float)
    return lower + rng.random(size=lower.shape) * (upper - lower)


def gauss_newton_slack_step(
    x: Array,
    g: Callable[[Array], Array],
    Jg: Optional[Callable[[Array], Array]] = None,
    h: Optional[Callable[[Array], Array]] = None,
    Jh: Optional[Callable[[Array], Array]] = None,
    finite_difference_step: float = 1e-6,
    regularization: float = 1e-8,
) -> Array:
    """
    Standalone Gauss-Newton slack step used by the direction-based restart
    seed strategy: for inequalities, only positive violations max(g(x),0)
    are included; for equalities, h(x) is included directly.
    """
    x = np.asarray(x, dtype=float)

    residuals = []
    jacobians = []

    gx = np.asarray(g(x), dtype=float).reshape(-1)
    active = gx > 0.0
    if np.any(active):
        Jg_x = (
            finite_difference_jacobian(g, x, finite_difference_step)
            if Jg is None
            else np.asarray(Jg(x), dtype=float)
        )
        residuals.append(gx[active])
        jacobians.append(Jg_x[active, :])

    if h is not None:
        hx = np.asarray(h(x), dtype=float).reshape(-1)
        if len(hx) > 0:
            Jh_x = (
                finite_difference_jacobian(h, x, finite_difference_step)
                if Jh is None
                else np.asarray(Jh(x), dtype=float)
            )
            residuals.append(hx)
            jacobians.append(Jh_x)

    if not residuals:
        return np.zeros_like(x)

    r = np.concatenate(residuals)
    J = np.vstack(jacobians)
    m = J.shape[0]

    A = J @ J.T + regularization * np.eye(m)
    dx = -J.T @ np.linalg.solve(A, r)
    return dx


def choose_restart_seed(
    lower: Array,
    upper: Array,
    rng: np.random.Generator,
    D: Optional[Array] = None,
    strategy: RestartStrategy = "uniform",
    candidates_per_restart: int = 50,
    slack_step: Optional[Callable[[Array], Array]] = None,
) -> Array:
    """
    Choose a restart seed using one of the three strategies in Sec. 3.3 of
    the paper:

    uniform:    sample x ~ U[l,u]
    distance:   sample candidates and choose the one furthest from D
    direction:  choose candidate whose slack step is least aligned with
                directions toward existing samples D

    D is the set of previously accepted samples, shape (num_samples, dim).
    """
    lower = np.asarray(lower, dtype=float)
    upper = np.asarray(upper, dtype=float)

    if strategy == "uniform" or D is None or len(D) == 0:
        return sample_uniform_box(lower, upper, rng)

    candidates = np.vstack([
        sample_uniform_box(lower, upper, rng)
        for _ in range(candidates_per_restart)
    ])
    D = np.asarray(D, dtype=float).reshape(-1, lower.size)

    if strategy == "distance":
        scores = []
        for x_i in candidates:
            distances = np.linalg.norm(D - x_i, axis=1)
            scores.append(np.min(distances))
        return candidates[int(np.argmax(scores))]

    if strategy == "direction":
        if slack_step is None:
            raise ValueError("direction restart seeding requires slack_step(x).")

        scores = []
        for x_i in candidates:
            delta_i = np.asarray(slack_step(x_i), dtype=float)
            delta_norm = np.linalg.norm(delta_i)

            if delta_norm < 1e-12:
                scores.append(-np.inf)
                continue

            dirs = D - x_i
            dir_norms = np.linalg.norm(dirs, axis=1)
            valid = dir_norms > 1e-12

            if not np.any(valid):
                scores.append(-np.inf)
                continue

            alignments = (dirs[valid] @ delta_i) / (dir_norms[valid] * delta_norm)
            scores.append(np.max(alignments))

        return candidates[int(np.argmin(scores))]

    raise ValueError(f"Unknown restart strategy: {strategy}")


def restarting_nhr_sample(
    phase1: Callable[[Array], Optional[Array]],
    g: Callable[[Array], Array],
    lower: Array,
    upper: Array,
    Jg: Optional[Callable[[Array], Array]] = None,
    f: Optional[Callable[[Array], float]] = None,
    extra_check: Optional[Callable[[Array], bool]] = None,
    nhr_options: Optional[NHROptions] = None,
    restart_options: Optional[RestartOptions] = None,
    slack_step: Optional[Callable[[Array], Array]] = None,
    h: Optional[Callable[[Array], Array]] = None,
    Jh: Optional[Callable[[Array], Array]] = None,
    equality_tol: float = 1e-4,
    project_samples_to_manifold: bool = False,
    projection_iters: int = 10,
) -> tuple[Array, list[Dict[str, Any]]]:
    """
    Outer restart loop around the single sampler entry point.

    phase1(seed) should return a feasible x0, or None if Phase I fails -
    e.g. run_downhill_phase1 for a native Gauss-Newton downhill Phase I
    with no IK dependency.

    slack_step(x) only needed for direction-based restart seeding.
    """
    if nhr_options is None:
        nhr_options = NHROptions(verbose=False)
    if restart_options is None:
        restart_options = RestartOptions()

    rng = np.random.default_rng(restart_options.random_seed)

    all_samples: List[Array] = []
    all_info: list[Dict[str, Any]] = []
    D = np.empty((0, len(lower)))

    for r in range(restart_options.num_restarts):
        seed = choose_restart_seed(
            lower=lower, upper=upper, rng=rng, D=D,
            strategy=restart_options.strategy,
            candidates_per_restart=restart_options.candidates_per_restart,
            slack_step=slack_step,
        )

        x0 = phase1(seed)
        if x0 is None:
            all_info.append({"restart": r, "status": "phase1_failed", "seed": seed})
            continue

        try:
            opts_r = NHROptions(**vars(nhr_options))
            opts_r.random_seed = int(rng.integers(0, 2**31 - 1))

            samples_r, diag_r = nhr_sample(
                x0=x0, g=g, lower=lower, upper=upper, Jg=Jg, f=f, extra_check=extra_check,
                options=opts_r, h=h, Jh=Jh, equality_tol=equality_tol,
                project_samples_to_manifold=project_samples_to_manifold, projection_iters=projection_iters,
            )
        except ValueError as e:
            all_info.append({"restart": r, "status": "nhr_failed", "seed": seed, "x0": x0, "error": str(e)})
            continue

        if len(samples_r) > 0:
            all_samples.append(samples_r)
            D = np.vstack([D, samples_r])

        all_info.append({
            "restart": r, "status": "success", "seed": seed, "x0": x0,
            "num_samples": len(samples_r), "diagnostics": diag_r,
        })

    if len(all_samples) == 0:
        return np.empty((0, len(lower))), all_info

    return np.vstack(all_samples), all_info


def restarting_nhr_sample_with_equalities(
    phase1: Callable[[Array], Optional[Array]],
    g: Callable[[Array], Array],
    h: Callable[[Array], Array],
    lower: Array,
    upper: Array,
    Jg: Optional[Callable[[Array], Array]] = None,
    Jh: Optional[Callable[[Array], Array]] = None,
    f: Optional[Callable[[Array], float]] = None,
    extra_check: Optional[Callable[[Array], bool]] = None,
    nhr_options: Optional[NHROptions] = None,
    restart_options: Optional[RestartOptions] = None,
    slack_step: Optional[Callable[[Array], Array]] = None,
    equality_tol: float = 1e-4,
    project_samples_to_manifold: bool = False,
    projection_iters: int = 10,
) -> tuple[Array, list[Dict[str, Any]]]:
    """Backward-compatible wrapper around the single equality-aware restart loop."""
    return restarting_nhr_sample(
        phase1=phase1, g=g, lower=lower, upper=upper, Jg=Jg, f=f, extra_check=extra_check,
        nhr_options=nhr_options, restart_options=restart_options, slack_step=slack_step,
        h=h, Jh=Jh, equality_tol=equality_tol,
        project_samples_to_manifold=project_samples_to_manifold, projection_iters=projection_iters,
    )


def per_restart_spread(
    samples: Array, restart_info: list[Dict[str, Any]], idx: int,
) -> list[Dict[str, Any]]:
    """
    For each successful restart, report the within-chain std/mean/range of
    samples[:, idx] for that restart's own chain.

    If within-chain std is tiny while across-restart means are spread over
    a wide range, the walk isn't mixing locally - the pooled histogram's
    apparent coverage is coming from restart diversity, not from the
    random walk itself.
    """
    offset = 0
    rows = []
    for info in restart_info:
        if info["status"] != "success":
            continue
        n = info["num_samples"]
        chain = samples[offset:offset + n, idx]
        offset += n
        if n == 0:
            continue
        rows.append({
            "restart": info["restart"], "n": n,
            "mean": float(np.mean(chain)), "std": float(np.std(chain)),
            "range": float(np.ptp(chain)),
        })
    return rows


# -----------------------------------------------------------------------------
# Manifold projection / superseded helpers
# -----------------------------------------------------------------------------

def slack_reduce_equalities(
    x: Array,
    h: Callable[[Array], Array],
    Jh: Optional[Callable[[Array], Array]] = None,
    lower: Optional[Array] = None,
    upper: Optional[Array] = None,
    finite_difference_step: float = 1e-6,
    regularization: float = 1e-8,
    max_iters: int = 10,
    tol: float = 1e-8,
) -> Array:
    """
    Gauss-Newton projection/slack-reduction step for equalities h(x)=0.

    Useful after sampling in an epsilon tube if you want samples closer to
    the exact manifold - used by nhr_sample's optional
    project_samples_to_manifold polish.

        dx = -J_h(x)^T @ (J_h(x)@J_h(x)^T + reg I)^(-1) @ h(x)
    """
    x = np.asarray(x, dtype=float).copy()

    for _ in range(max_iters):
        hx = np.asarray(h(x), dtype=float).reshape(-1)

        if np.linalg.norm(hx, ord=np.inf) <= tol:
            break

        Jh_x = (
            finite_difference_jacobian(h, x, finite_difference_step)
            if Jh is None
            else np.asarray(Jh(x), dtype=float)
        )
        if Jh_x.ndim == 1:
            Jh_x = Jh_x.reshape(1, -1)

        m = Jh_x.shape[0]
        A = Jh_x @ Jh_x.T + regularization * np.eye(m)
        dx = -Jh_x.T @ np.linalg.solve(A, hx)
        x = x + dx

        if lower is not None:
            x = np.maximum(x, np.asarray(lower, dtype=float))
        if upper is not None:
            x = np.minimum(x, np.asarray(upper, dtype=float))

    return x


def make_tangent_direction_projector(
    h: Callable[[Array], Array],
    Jh: Optional[Callable[[Array], Array]] = None,
    finite_difference_step: float = 1e-6,
    regularization: float = 1e-8,
) -> Callable[[Array, Array], Array]:
    """
    Returns direction_projector(x, d) that projects d into the tangent
    space of h(x)=0.

    SUPERSEDED: nhr_sample/restarting_nhr_sample no longer take a
    direction_projector hook - step_hit_and_run always projects tangent to
    h internally (sample_random_tangent_direction), matching the
    reference. Kept here only for ad hoc experimentation.
    """
    def direction_projector(x: Array, d: Array) -> Array:
        Jh_x = (
            finite_difference_jacobian(h, x, finite_difference_step)
            if Jh is None
            else np.asarray(Jh(x), dtype=float)
        )
        return project_direction_to_equality_tangent(d=d, Jh_x=Jh_x, regularization=regularization)

    return direction_projector


def make_active_set_direction_projector(
    g: Callable[[Array], Array],
    Jg: Optional[Callable[[Array], Array]] = None,
    h: Optional[Callable[[Array], Array]] = None,
    Jh: Optional[Callable[[Array], Array]] = None,
    active_tol: float = 1e-3,
    finite_difference_step: float = 1e-6,
    regularization: float = 1e-8,
) -> Callable[[Array, Array], Array]:
    """
    Returns direction_projector(x, d) that projects d tangent to h(x)=0
    AND tangent to whichever rows of g(x)<=0 are currently within
    active_tol of their boundary.

    SUPERSEDED: this was a heuristic bolted onto the old strict-accept
    step to survive very tight inequalities. The new step_hit_and_run's
    lenient accept criterion solves the same problem structurally, so this
    is no longer wired into nhr_sample/restarting_nhr_sample. Kept here
    only for ad hoc experimentation.
    """
    def direction_projector(x: Array, d: Array) -> Array:
        rows = []

        if h is not None:
            Jh_x = (
                finite_difference_jacobian(h, x, finite_difference_step)
                if Jh is None
                else np.asarray(Jh(x), dtype=float)
            )
            if Jh_x.size:
                rows.append(np.atleast_2d(Jh_x))

        gx = np.asarray(g(x), dtype=float).reshape(-1)
        active = gx >= -active_tol
        if np.any(active):
            Jg_x = (
                finite_difference_jacobian(g, x, finite_difference_step)
                if Jg is None
                else np.asarray(Jg(x), dtype=float)
            )
            rows.append(np.atleast_2d(Jg_x)[active, :])

        if not rows:
            return d

        J_active = np.vstack(rows)
        return project_direction_to_equality_tangent(d=d, Jh_x=J_active, regularization=regularization)

    return direction_projector


def make_gauss_newton_corrector(
    g: Callable[[Array], Array],
    Jg: Optional[Callable[[Array], Array]] = None,
    h: Optional[Callable[[Array], Array]] = None,
    Jh: Optional[Callable[[Array], Array]] = None,
    iters: int = 2,
    finite_difference_step: float = 1e-6,
    regularization: float = 1e-8,
) -> Callable[[Array], Array]:
    """
    Returns corrector(y) that nudges a candidate point toward feasibility
    using a few Gauss-Newton slack steps.

    SUPERSEDED: this was a per-micro-step retraction bolted onto the old
    strict-accept step. run_interior's conditional step_gauss_newton
    cleanup now does this at the right granularity (once per outer
    iteration, only when needed). Kept here only for ad hoc experimentation.
    """
    def corrector(y: Array) -> Array:
        y = np.asarray(y, dtype=float).copy()
        for _ in range(iters):
            dx = gauss_newton_slack_step(
                x=y, g=g, Jg=Jg, h=h, Jh=Jh,
                finite_difference_step=finite_difference_step, regularization=regularization,
            )
            if not np.any(dx):
                break
            y = y + dx
        return y

    return corrector


# -----------------------------------------------------------------------------
# Plotting and saving artifacts
# -----------------------------------------------------------------------------

def save_joint_sample_artifacts(
    samples: Array,
    diagnostics: list[Dict[str, Any]],
    lower: Optional[Array] = None,
    upper: Optional[Array] = None,
    joint_names: Optional[List[str]] = None,
    options: Optional[NHROptions] = None,
    output_root: str | Path = "joint_samples_plots",
    note: Optional[str] = None,
    timestamp: Optional[str] = None,
    bins: int = 30,
    show: bool = True,
) -> Path:
    """
    Save per-joint histograms, a Gaussian fit overlay, and a JSON debug report.

    The output layout is:

        joint_samples_plots/<timestamp>/
            joint_name.png
            info.json

    Parameters
    ----------
    samples:
        Sample matrix with shape (num_samples, num_joints).
    diagnostics:
        Step-by-step diagnostics returned by `nhr_sample` or
        `restarting_nhr_sample`.
    lower, upper:
        Optional joint limits to draw as vertical lines.
    joint_names:
        Optional list of names for the histogram titles and filenames.
    options:
        Optional `NHROptions` instance to include in the report.
    output_root:
        Root directory used to store timestamped outputs.
    note:
        Free-form note to include in the report.
    timestamp:
        Optional run label. If omitted, the current time is used.
    bins:
        Histogram bin count.
    show:
        If True, display each figure inline with `plt.show()`.
    """

    import matplotlib.pyplot as plt
    from scipy.stats import norm

    samples = np.asarray(samples, dtype=float)
    if samples.ndim != 2:
        raise ValueError(f"Expected samples to be a 2D array, got shape {samples.shape}.")

    num_samples, num_joints = samples.shape
    if joint_names is None:
        joint_names = [f"q{i}" for i in range(num_joints)]
    elif len(joint_names) != num_joints:
        raise ValueError(
            f"joint_names has length {len(joint_names)} but samples has {num_joints} columns."
        )

    if lower is not None:
        lower = np.asarray(lower, dtype=float).reshape(-1)
    if upper is not None:
        upper = np.asarray(upper, dtype=float).reshape(-1)

    if lower is not None and len(lower) != num_joints:
        raise ValueError(f"lower has length {len(lower)} but samples has {num_joints} columns.")
    if upper is not None and len(upper) != num_joints:
        raise ValueError(f"upper has length {len(upper)} but samples has {num_joints} columns.")

    run_stamp = timestamp or datetime.now().strftime("%Y%m%d_%H%M%S")
    output_root = Path(output_root)
    run_dir = output_root / run_stamp
    run_dir.mkdir(parents=True, exist_ok=True)

    def safe_filename(name: str) -> str:
        cleaned = "".join(ch if ch.isalnum() or ch in "._-" else "_" for ch in name)
        return cleaned or "joint"

    joint_reports = []

    for idx, joint_name in enumerate(joint_names):
        values = samples[:, idx]
        mean = float(np.mean(values))
        std = float(np.std(values, ddof=1)) if num_samples > 1 else 0.0

        fig, ax = plt.subplots(figsize=(8, 5))
        _, bin_edges, _ = ax.hist(
            values,
            bins=bins,
            density=False,
            alpha=0.7,
            color="#4c78a8",
            edgecolor="white",
            label="Samples",
        )

        if std > 0 and len(bin_edges) > 1:
            x = np.linspace(bin_edges[0], bin_edges[-1], 400)
            bin_width = bin_edges[1] - bin_edges[0]
            pdf = norm.pdf(x, loc=mean, scale=std) * len(values) * bin_width
            ax.plot(x, pdf, color="#f58518", linewidth=2.5, label="Gaussian fit")

        ax.axvline(mean, color="#54a24b", linestyle="--", linewidth=2, label=f"Mean = {mean:.4f}")
        ax.axvline(mean - std, color="#72b7b2", linestyle=":", linewidth=2, label=f"Std = {std:.4f}")
        ax.axvline(mean + std, color="#72b7b2", linestyle=":", linewidth=2)

        lower_limit = None if lower is None else float(lower[idx])
        upper_limit = None if upper is None else float(upper[idx])

        if lower_limit is not None:
            ax.axvline(
                lower_limit,
                color="#e45756",
                linestyle="-.",
                linewidth=2,
                label=f"Lower limit = {lower_limit:.4f}",
            )
        if upper_limit is not None:
            ax.axvline(
                upper_limit,
                color="#b279a2",
                linestyle="-.",
                linewidth=2,
                label=f"Upper limit = {upper_limit:.4f}",
            )

        ax.set_title(f"Joint Samples: {joint_name}")
        ax.set_xlabel("Joint value")
        ax.set_ylabel("Frequency")
        ax.grid(True, alpha=0.25)
        ax.legend(loc="best", fontsize=9)
        fig.tight_layout()

        file_name = f"{safe_filename(joint_name)}.png"
        fig.savefig(run_dir / file_name, dpi=200, bbox_inches="tight")
        if show:
            plt.show()
        plt.close(fig)

        joint_reports.append({
            "joint_index": idx,
            "joint_name": joint_name,
            "file_name": file_name,
            "mean": mean,
            "std": std,
            "min": float(np.min(values)),
            "max": float(np.max(values)),
            "lower_limit": lower_limit,
            "upper_limit": upper_limit,
        })

    info = {
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "run_stamp": run_stamp,
        "output_root": str(output_root),
        "run_dir": str(run_dir),
        "note": note,
        "num_samples": int(num_samples),
        "num_joints": int(num_joints),
        "joint_names": joint_names,
        "options": _json_safe(vars(options)) if options is not None else None,
        "sample_summary": {
            "mean": np.mean(samples, axis=0),
            "std": np.std(samples, axis=0, ddof=1) if num_samples > 1 else np.zeros(num_joints),
            "min": np.min(samples, axis=0),
            "max": np.max(samples, axis=0),
        },
        "limits": {
            "lower": lower,
            "upper": upper,
        },
        "joint_reports": joint_reports,
        "diagnostics": _json_safe(diagnostics),
    }

    with open(run_dir / "info.json", "w", encoding="utf-8") as f:
        json.dump(_json_safe(info), f, indent=2, sort_keys=True)

    if note:
        print(f"NHR note: {note}")
    print(f"Saved joint sample artifacts to {run_dir}")

    return run_dir
