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


@dataclass
class NHROptions:
    num_samples: int = 1000
    burn_in: int = 100
    thinning: int = 1

    delta_max: float = 0.1
    constraint_tol: float = 1e-8

    max_inner_tries: int = 50
    use_initial_linear_clip: bool = True

    finite_difference_step: float = 1e-6
    random_seed: int = 0

    verbose: bool = True


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

    This is the core hit-and-run clipping operation.
    """
    g_bar = np.asarray(g_bar, dtype=float).reshape(-1)
    a = np.asarray(a, dtype=float).reshape(-1)

    for gi, ai in zip(g_bar, a): # iterate over each linear inequality
        if abs(ai) < tol:
            # Constraint does not depend on beta.
            # If already violated, interval becomes empty.
            
            if gi > 0: # if a is 0, and g_bar > 0, then the constraint is violated already
                return 1.0, 0.0     # so we return an empty interval (beta_low > beta_up)
            continue

        beta_boundary = -gi / ai  # beta_i = -gi / ai

        if ai > 0:
            # Need beta <= beta_boundary. 
            # beta_up <= -gi / ai
            # upper bound must be <= beta_boundary
            beta_up = min(beta_up, beta_boundary)
        else: # ai < 0
            # Need beta >= beta_boundary.
            # beta_lo >= -gi / ai
            # lower bound must be >= beta_boundary
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
    Clip beta interval so that:

        lower <= x + beta d <= upper
        
        this is equivalent to two linear inequalities of the form (g_bar) + beta (a) <= 0:
        
        1. (lower - x) + beta (-d) <= 0
        2. (x - upper) + beta (d) <= 0
        
    "along this random direction d, starting from x, how far can we go before hitting the bounds for any of the variables?"
        
    """
    x = np.asarray(x, dtype=float)
    d = np.asarray(d, dtype=float)

    # lower bound: (lower - x) + beta (-d) <= 0
    beta_lo, beta_up = clip_interval_with_linear_ineq(
        beta_lo,
        beta_up,
        g_bar=lower - x,
        a=-d,
    )

    # upper bound: (x - upper) + beta (d) <= 0
    beta_lo, beta_up = clip_interval_with_linear_ineq(
        beta_lo,
        beta_up,
        g_bar=x - upper,
        a=d,
    )

    return beta_lo, beta_up


def is_feasible(
    x: Array,
    g: Callable[[Array], Array],
    constraint_tol: float = 1e-8,
    extra_check: Optional[Callable[[Array], bool]] = None,
) -> bool:
    """
    Feasible means:
        g(x) <= constraint_tol
    plus optional extra boolean checks.
    """
    gx = np.asarray(g(x), dtype=float)

    if np.any(gx > constraint_tol):
        return False

    if extra_check is not None and not extra_check(x):
        return False

    return True


def nonlinear_hit_and_run_step(
    x: Array,
    g: Callable[[Array], Array],
    Jg: Optional[Callable[[Array], Array]],
    f: Callable[[Array], float],
    lower: Array,
    upper: Array,
    rng: np.random.Generator,
    options: NHROptions,
    extra_check: Optional[Callable[[Array], bool]] = None,
    direction_projector: Optional[Callable[[Array, Array], Array]] = None,
) -> tuple[Array, Dict[str, Any]]:
    """
    One Nonlinear Metropolis-Adjusted Hit-and-Run step.

    This follows Algorithm 3 from Toussaint et al.:

    1. sample random direction d
    2. initialise beta interval [-delta_max, delta_max]
    3. clip interval using box bounds
    4. optionally clip using linearisation at current x
    5. sample candidate y = x + beta d
    6. if y feasible, accept/reject using MH
    7. if y infeasible, linearise violated constraints at y and shrink interval
    """

    x = np.asarray(x, dtype=float)
    dim = len(x)

    if Jg is None:
        Jg_eval = lambda z: finite_difference_jacobian(
            g, z, step=options.finite_difference_step
        )
    else:
        Jg_eval = Jg

    # Algorithm 3, line 2: random direction.
    d = rng.normal(size=dim)

    # Optional equality handling: project the random direction into
    # the tangent space of h(x)=0 before normalising it.
    if direction_projector is not None:
        d = np.asarray(direction_projector(x, d), dtype=float)

    d_norm = np.linalg.norm(d)

    if d_norm < 1e-12:
        return x, {"accepted": False, "reason": "zero_direction"}

    d = d / d_norm

    # Algorithm 3, line 3.
    beta_lo = -options.delta_max
    beta_up = options.delta_max

    # Algorithm 3, line 4: clip with box bounds.
    # for clipping on the variable bounds lower <= x <= upper
    beta_lo, beta_up = clip_interval_with_box_bounds(
        x=x,
        d=d,
        lower=lower,
        upper=upper,
        beta_lo=beta_lo,
        beta_up=beta_up,
    )

    # Algorithm 3, line 5: optional clip with linearisation at x.
    # for clipping on the nonlinear inequalities g(x) <= 0
    if options.use_initial_linear_clip and beta_lo <= beta_up:
        gx = np.asarray(g(x), dtype=float)
        Gx = np.asarray(Jg_eval(x), dtype=float)
        ax = Gx @ d


        # g(x + beta d) ≈ g(x) + Jg(x) (x + beta d) <= 0
        # (g(x)) + beta (Jg(x) d) <= 0
        beta_lo, beta_up = clip_interval_with_linear_ineq(
            beta_lo=beta_lo,
            beta_up=beta_up,
            g_bar=gx,
            a=ax,
        )

    # Algorithm 3, line 6: inner loop.
    for inner_try in range(options.max_inner_tries):

        # Algorithm 3, line 7.
        if beta_lo > beta_up:
            return x, {
                "accepted": False,
                "reason": "empty_interval",
                "inner_try": inner_try,
            }

        # Algorithm 3, line 8.
        beta = rng.uniform(beta_lo, beta_up)
        y = x + beta * d

        gy = np.asarray(g(y), dtype=float)

        # Boolean checks that are not differentiable, e.g. preliminary grasp checker.
        passes_extra = True
        if extra_check is not None:
            passes_extra = bool(extra_check(y))

        # Algorithm 3, line 10.
        if np.all(gy <= options.constraint_tol) and passes_extra:
            fx = float(f(x))
            fy = float(f(y))

            # Algorithm 3, line 11:
            # accept with probability min(1, exp(-f(y)) / exp(-f(x))).
            #
            # In log form:
            # accept if log(u) < f(x) - f(y).
            log_accept_ratio = fx - fy


            # lower (better) energy f(y) means higher acceptance probability.
            if np.log(rng.uniform()) < min(0.0, log_accept_ratio):
                return y, {
                    "accepted": True,
                    "reason": "mh_accept",
                    "inner_try": inner_try,
                    "beta": beta,
                    "f_old": fx,
                    "f_new": fy,
                }

            return x, {
                "accepted": False,
                "reason": "mh_reject",
                "inner_try": inner_try,
                "beta": beta,
                "f_old": fx,
                "f_new": fy,
            }

        # Algorithm 3, line 14:
        # If nonlinear inequalities are violated, linearise at y and shrink interval.
        #
        # Linearisation around y:
        #   g(x + beta d) ≈ g(y) + Jg(y) (x + beta d - y)
        #                 = [g(y) + Jg(y)(x - y)] + beta [Jg(y)d]
        violated = gy > options.constraint_tol

        if np.any(violated):
            Gy = np.asarray(Jg_eval(y), dtype=float)

            g_bar = gy[violated] + Gy[violated, :] @ (x - y)
            a = Gy[violated, :] @ d

            beta_lo, beta_up = clip_interval_with_linear_ineq(
                beta_lo=beta_lo,
                beta_up=beta_up,
                g_bar=g_bar,
                a=a,
            )
        else:
            # This case means g is okay but extra_check failed.
            # Since extra_check has no gradient, we cannot clip intelligently.
            # We simply keep trying along the same interval.
            continue

    return x, {
        "accepted": False,
        "reason": "max_inner_tries",
        "inner_try": options.max_inner_tries,
    }


def nhr_sample(
    x0: Array,
    g: Callable[[Array], Array],
    lower: Array,
    upper: Array,
    Jg: Optional[Callable[[Array], Array]] = None,
    f: Optional[Callable[[Array], float]] = None,
    extra_check: Optional[Callable[[Array], bool]] = None,
    options: Optional[NHROptions] = None,
    direction_projector: Optional[Callable[[Array, Array], Array]] = None,
    h: Optional[Callable[[Array], Array]] = None,
    Jh: Optional[Callable[[Array], Array]] = None,
    equality_tol: float = 1e-4,
    project_samples_to_manifold: bool = False,
    projection_iters: int = 10,
) -> tuple[Array, list[Dict[str, Any]]]:
    """
    Run Nonlinear Metropolis-Adjusted Hit-and-Run.

    This is the single public entry point for sampling with or without
    equality constraints. When equalities are provided via ``h`` and ``Jh``,
    the implementation first converts them into an epsilon-tube of
    inequalities, projects proposal directions onto the local tangent space,
    and can optionally project the returned samples back onto the manifold.

    Parameters
    ----------
    x0:
        Initial feasible point. For your project this could be q_full from IK.

    g:
        Function returning all nonlinear inequalities.
        Feasible means g(x) <= 0.

    lower, upper:
        Box bounds on x.

    Jg:
        Jacobian of g. Shape: (num_constraints, dim).
        If None, finite differences are used.

    f:
        Energy/cost. If None, uniform sampling over feasible set is used.

    extra_check:
        Optional non-differentiable boolean checker.
        Example: grasp_checker(x), collision_checker(x).
        Use this for prototyping only; if possible, convert checks to smooth g(x).

    options:
        NHROptions.

    h:
        Optional equality constraint function with h(x) = 0.

    Jh:
        Optional Jacobian of h. If None, finite differences are used.

    equality_tol:
        Tube width used to turn h(x)=0 into h(x) in [-tol, tol].

    project_samples_to_manifold:
        If True, project each accepted sample back near h(x)=0 with a
        Gauss-Newton step after sampling.

    projection_iters:
        Number of Gauss-Newton iterations used for optional projection.

    Returns
    -------
    samples:
        Array of feasible samples, shape (num_samples, dim).

    diagnostics:
        List of dictionaries describing each NHR step.
    """

    if options is None:
        options = NHROptions()

    if f is None:
        f = lambda x: 0.0

    x = np.asarray(x0, dtype=float).copy()
    lower = np.asarray(lower, dtype=float)
    upper = np.asarray(upper, dtype=float)

    if x.shape != lower.shape or x.shape != upper.shape:
        raise ValueError("x0, lower, and upper must have the same shape.")

    if not np.all(lower <= upper):
        raise ValueError("All lower bounds must be <= upper bounds.")

    if not np.all((x >= lower) & (x <= upper)):
        raise ValueError("x0 must be inside the box bounds.")

    # Build the inequality/equality evaluation path once. If no equalities are
    # supplied, the sampler runs exactly as before. Otherwise, the equalities
    # are converted into a margin tube so the existing feasibility checks can
    # be reused without branching inside the main loop.
    if h is None:
        g_eval = g
        J_eval = Jg
        active_direction_projector = direction_projector
    else:
        g_eval = make_equality_tube_inequalities(g, h, equality_tol)
        J_eval = make_equality_tube_jacobian(
            Jg=Jg,
            Jh=Jh,
            g=g,
            h=h,
            finite_difference_step=options.finite_difference_step,
        )
        if direction_projector is None:
            active_direction_projector = make_tangent_direction_projector(
                h=h,
                Jh=Jh,
                finite_difference_step=options.finite_difference_step,
            )
        else:
            active_direction_projector = direction_projector

    if not is_feasible(
        x,
        g=g_eval,
        constraint_tol=options.constraint_tol,
        extra_check=extra_check,
    ):
        raise ValueError(
            "x0 is not feasible. Run IK / Phase-I slack minimisation first."
        )

    rng = np.random.default_rng(options.random_seed)

    samples = []
    diagnostics = []

    total_steps = options.burn_in + options.num_samples * options.thinning

    for step in range(total_steps):
        x, info = nonlinear_hit_and_run_step(
            x=x,
            g=g_eval,
            Jg=J_eval,
            f=f,
            lower=lower,
            upper=upper,
            rng=rng,
            options=options,
            extra_check=extra_check,
            direction_projector=active_direction_projector,
        )

        info["step"] = step
        info["is_burn_in"] = step < options.burn_in
        diagnostics.append(info)

        if step >= options.burn_in:
            if (step - options.burn_in) % options.thinning == 0:
                if is_feasible(
                    x,
                    g=g_eval,
                    constraint_tol=options.constraint_tol,
                    extra_check=extra_check,
                ):
                    samples.append(x.copy())

        if options.verbose and step % max(1, total_steps // 10) == 0:
            print(
                f"step {step:05d}/{total_steps} | "
                f"samples {len(samples):05d}/{options.num_samples} | "
                f"last: {info['reason']}"
            )

    if project_samples_to_manifold and h is not None and len(samples) > 0:
        # After sampling in the equality tube, project the accepted points back
        # toward the exact manifold h(x)=0. This keeps the returned samples
        # aligned with the equalities without changing the main sampler loop.
        projected = []
        for x in samples:
            xp = slack_reduce_equalities(
                x=x,
                h=h,
                Jh=Jh,
                lower=lower,
                upper=upper,
                finite_difference_step=options.finite_difference_step,
                max_iters=projection_iters,
                tol=options.constraint_tol,
            )
            if is_feasible(xp, g_eval, options.constraint_tol, extra_check):
                projected.append(xp)
        samples = np.asarray(projected)

    return np.asarray(samples), diagnostics

# -----------------------------------------------------------------------------
# Equality handling for NHR
# -----------------------------------------------------------------------------

def make_equality_tube_inequalities(
    g: Callable[[Array], Array],
    h: Callable[[Array], Array],
    equality_tol: float,
) -> Callable[[Array], Array]:
    """
    Convert equalities h(x)=0 into epsilon-margin inequalities:

        h(x) - eps <= 0
       -h(x) - eps <= 0

    and concatenate them with the original inequalities g(x)<=0.
    """
    def g_total(x: Array) -> Array:
        gx = np.asarray(g(x), dtype=float).reshape(-1)
        hx = np.asarray(h(x), dtype=float).reshape(-1)
        return np.concatenate([gx, hx - equality_tol, -hx - equality_tol])

    return g_total


def make_equality_tube_jacobian(
    Jg: Optional[Callable[[Array], Array]],
    Jh: Optional[Callable[[Array], Array]],
    g: Callable[[Array], Array],
    h: Callable[[Array], Array],
    finite_difference_step: float = 1e-6,
) -> Callable[[Array], Array]:
    """
    Jacobian for the concatenated inequality vector
    [g(x), h(x)-eps, -h(x)-eps].
    """
    def J_total(x: Array) -> Array:
        Jg_x = (
            finite_difference_jacobian(g, x, finite_difference_step)
            if Jg is None
            else np.asarray(Jg(x), dtype=float)
        )
        Jh_x = (
            finite_difference_jacobian(h, x, finite_difference_step)
            if Jh is None
            else np.asarray(Jh(x), dtype=float)
        )
        return np.vstack([Jg_x, Jh_x, -Jh_x])

    return J_total


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

    This removes the part of d that immediately changes the equalities.
    """
    d = np.asarray(d, dtype=float).reshape(-1)
    Jh_x = np.asarray(Jh_x, dtype=float)

    # If there are no equalities, the tangent space is the whole space.
    if Jh_x.size == 0:
        return d

    # If there is only one equality, make sure Jh_x is 2D for the matrix operations.
    if Jh_x.ndim == 1:
        Jh_x = Jh_x.reshape(1, -1)

    m = Jh_x.shape[0] # number of equalities
    
    A = Jh_x @ Jh_x.T + regularization * np.eye(m) # (J_h @ J_h^T + reg I)
    
    # np.linalg.solve(A, Jh_x @ d) solves A y = Jh_x @ d for y
    # y = (J_h J_h^T + reg I)^(-1) J_h d
    # normal_component = J_h^T (J_h J_h^T + reg I)^(-1) J_h d
    normal_component = Jh_x.T @ np.linalg.solve(A, Jh_x @ d) 
    
    # tangent_component = d - normal_component
    return d - normal_component # d_tan = (I - P_h) * d = d - P_h * d


def make_tangent_direction_projector(
    h: Callable[[Array], Array],
    Jh: Optional[Callable[[Array], Array]] = None,
    finite_difference_step: float = 1e-6,
    regularization: float = 1e-8,
) -> Callable[[Array, Array], Array]:
    """
    Returns a function direction_projector(x, d) that projects d into
    the tangent space of h(x)=0.
    """
    def direction_projector(x: Array, d: Array) -> Array:
        Jh_x = (
            finite_difference_jacobian(h, x, finite_difference_step)
            if Jh is None
            else np.asarray(Jh(x), dtype=float)
        )
        return project_direction_to_equality_tangent(
            d=d,
            Jh_x=Jh_x,
            regularization=regularization,
        )

    return direction_projector


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

    This is useful after sampling in an epsilon tube if you want samples
    closer to the exact manifold.
    
    This corresponds to moving in the direction of the negative derivative of F_01(x) = s(x)^T*s(x) 
    where s(x) is the slack vector for constraint violation.
        
        s(x) = (RELU(g(x)), abs(h(x)))
    
    since NHR already handles inequalities, we only need to reduce the slack for equalities h(x)=0. So:
    
        s(x) = abs(h(x))
    
    The update is either through Gradient Descent or Gauss-Newton. Here, we use Gauss-Newton (geometry aware):
    Gauss-Newton step:
        −α * (∇^2F(x) + λI)^(-1) @ ∇F(x)
        
        where ∇^2F(x) is the Hessian of F(x) and λI is the regularisation term to ensure positive definiteness.
        
        ∇^2F(x) ≈ J_h(x)^T @ J_h(x)  (Gauss-Newton approximation)
        ∇F(x) = J_h(x)^T @ h(x)
        
        so the update becomes:
        dx = -α * (J_h(x)^T @ J_h(x) + λI)^(-1) @ J_h(x)^T @ h(x)
    """
    x = np.asarray(x, dtype=float).copy()

    for _ in range(max_iters):
        hx = np.asarray(h(x), dtype=float).reshape(-1)
        
        # If the equalities are already satisfied within tolerance, we can stop early.
        if np.linalg.norm(hx, ord=np.inf) <= tol:
            break

        # Compute Jacobian of h at x, either using provided Jh or finite differences.
        Jh_x = (
            finite_difference_jacobian(h, x, finite_difference_step)
            if Jh is None
            else np.asarray(Jh(x), dtype=float)
        )
        if Jh_x.ndim == 1:
            Jh_x = Jh_x.reshape(1, -1)


        m = Jh_x.shape[0] # number of equalities
        
        # Compute the projection matrix P_h = J_h^T (J_h J_h^T + reg I)^(-1) J_h = J_h^T A^{-1} J_h
        A = Jh_x @ Jh_x.T + regularization * np.eye(m)
        
        # Solve A y = h(x) for y, then compute the update dx = -J_h^T y = -J_h^T @ A^{-1} @ h(x)
        dx = -Jh_x.T @ np.linalg.solve(A, hx)
        x = x + dx

        if lower is not None or upper is not None:
            if lower is not None:
                x = np.maximum(x, np.asarray(lower, dtype=float))
            if upper is not None:
                x = np.minimum(x, np.asarray(upper, dtype=float))

    return x


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
    """
    Backward-compatible wrapper around the single equality-aware sampler.

    The implementation now lives in ``nhr_sample`` so that the equality path is
    managed in one place with a shared comment block and a single set of
    optional arguments.
    """
    return nhr_sample(
        x0=x0,
        g=g,
        lower=lower,
        upper=upper,
        Jg=Jg,
        f=f,
        extra_check=extra_check,
        options=options,
        h=h,
        Jh=Jh,
        equality_tol=equality_tol,
        project_samples_to_manifold=project_samples_to_manifold,
        projection_iters=projection_iters,
    )


# -----------------------------------------------------------------------------
# Restart seeding
# -----------------------------------------------------------------------------

RestartStrategy = Literal["uniform", "distance", "direction"]


@dataclass
class RestartOptions:
    num_restarts: int = 10
    strategy: RestartStrategy = "uniform"
    candidates_per_restart: int = 50
    random_seed: int = 0
    keep_failed_phase1_seeds: bool = False


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
    Prototype Gauss-Newton slack step used by the direction-based restart seed.

    For inequalities, only positive violations max(g(x),0) are included.
    For equalities, h(x) is included directly.
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

    # Minimum-norm Gauss-Newton correction for J dx ≈ -r.
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
    Choose a restart seed using one of the three strategies in Sec. 3.3:

    uniform:    sample x ~ U[l,u]
    distance:   sample candidates and choose the one furthest from D
    direction:  choose candidate whose slack step is least aligned with
                directions toward existing samples D
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
        # score_i = min_y ||y - x_i||. Choose max score.
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
                # No predicted movement; prefer it only if far from data.
                scores.append(-np.inf)
                continue

            dirs = D - x_i
            dir_norms = np.linalg.norm(dirs, axis=1)
            valid = dir_norms > 1e-12

            if not np.any(valid):
                scores.append(-np.inf)
                continue

            alignments = (dirs[valid] @ delta_i) / (dir_norms[valid] * delta_norm)
            # Paper: choose argmin_i max_y alignment.
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

    phase1(seed) should implement your IK/slack-reduction phase and return
    a feasible x0, or None if Phase I fails.
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
            lower=lower,
            upper=upper,
            rng=rng,
            D=D,
            strategy=restart_options.strategy,
            candidates_per_restart=restart_options.candidates_per_restart,
            slack_step=slack_step,
        )

        x0 = phase1(seed)
        if x0 is None:
            all_info.append({"restart": r, "status": "phase1_failed", "seed": seed})
            continue

        try:
            # Make each chain reproducible but different.
            opts_r = NHROptions(**vars(nhr_options))
            opts_r.random_seed = int(rng.integers(0, 2**31 - 1))

            samples_r, diag_r = nhr_sample(
                x0=x0,
                g=g,
                lower=lower,
                upper=upper,
                Jg=Jg,
                f=f,
                extra_check=extra_check,
                options=opts_r,
                h=h,
                Jh=Jh,
                equality_tol=equality_tol,
                project_samples_to_manifold=project_samples_to_manifold,
                projection_iters=projection_iters,
            )
        except ValueError as e:
            all_info.append({
                "restart": r,
                "status": "nhr_failed",
                "seed": seed,
                "x0": x0,
                "error": str(e),
            })
            continue

        if len(samples_r) > 0:
            all_samples.append(samples_r)
            D = np.vstack([D, samples_r])

        all_info.append({
            "restart": r,
            "status": "success",
            "seed": seed,
            "x0": x0,
            "num_samples": len(samples_r),
            "diagnostics": diag_r,
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
    """
    Backward-compatible wrapper around the single equality-aware restart loop.
    """
    return restarting_nhr_sample(
        phase1=phase1,
        g=g,
        lower=lower,
        upper=upper,
        Jg=Jg,
        f=f,
        extra_check=extra_check,
        nhr_options=nhr_options,
        restart_options=restart_options,
        slack_step=slack_step,
        h=h,
        Jh=Jh,
        equality_tol=equality_tol,
        project_samples_to_manifold=project_samples_to_manifold,
        projection_iters=projection_iters,
    )




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
