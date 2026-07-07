import numpy as np
from dataclasses import dataclass
from typing import Callable, Optional, Dict, Any


Array = np.ndarray


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

    for gi, ai in zip(g_bar, a):
        if abs(ai) < tol:
            # Constraint does not depend on beta.
            # If already violated, interval becomes empty.
            if gi > 0:
                return 1.0, 0.0
            continue

        beta_boundary = -gi / ai

        if ai > 0:
            # Need beta <= beta_boundary.
            beta_up = min(beta_up, beta_boundary)
        else:
            # Need beta >= beta_boundary.
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
    """
    x = np.asarray(x, dtype=float)
    d = np.asarray(d, dtype=float)

    # lower bound: lower - x - beta d <= 0
    beta_lo, beta_up = clip_interval_with_linear_ineq(
        beta_lo,
        beta_up,
        g_bar=lower - x,
        a=-d,
    )

    # upper bound: x - upper + beta d <= 0
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
    d_norm = np.linalg.norm(d)

    if d_norm < 1e-12:
        return x, {"accepted": False, "reason": "zero_direction"}

    d = d / d_norm

    # Algorithm 3, line 3.
    beta_lo = -options.delta_max
    beta_up = options.delta_max

    # Algorithm 3, line 4: clip with box bounds.
    beta_lo, beta_up = clip_interval_with_box_bounds(
        x=x,
        d=d,
        lower=lower,
        upper=upper,
        beta_lo=beta_lo,
        beta_up=beta_up,
    )

    # Algorithm 3, line 5: optional clip with linearisation at x.
    if options.use_initial_linear_clip and beta_lo <= beta_up:
        gx = np.asarray(g(x), dtype=float)
        Gx = np.asarray(Jg_eval(x), dtype=float)
        ax = Gx @ d

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
) -> tuple[Array, list[Dict[str, Any]]]:
    """
    Run Nonlinear Metropolis-Adjusted Hit-and-Run.

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

    if not is_feasible(
        x,
        g=g,
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
            g=g,
            Jg=Jg,
            f=f,
            lower=lower,
            upper=upper,
            rng=rng,
            options=options,
            extra_check=extra_check,
        )

        info["step"] = step
        info["is_burn_in"] = step < options.burn_in
        diagnostics.append(info)

        if step >= options.burn_in:
            if (step - options.burn_in) % options.thinning == 0:
                if is_feasible(
                    x,
                    g=g,
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

    return np.asarray(samples), diagnostics