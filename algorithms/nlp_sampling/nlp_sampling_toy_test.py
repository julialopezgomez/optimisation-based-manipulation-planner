"""
Drake-free synthetic validation of nlp_sampling.run_and_report - this repo's
CI can't run the Panda+cap grasp problem in nlp_sampling_standalone_test.py
(it needs pydrake), so run_and_report's "sample -> diagnose -> save" wiring
is instead exercised here against a toy constrained-sampling problem with
plain analytic g/Jg/h/Jh, no plant construction of any kind.

Toy problem: 2D points in the box [-1, 1]^2, constrained to the closed unit
disk (inequality) intersected with the line x0 + x1 = 0 (equality) - i.e.
sampling the diameter segment of the unit disk that runs through the origin
at -45 degrees.

Usage:
    python nlp_sampling_toy_test.py
"""

import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import nlp_sampling


def g_disk(x):
    return np.array([x[0] ** 2 + x[1] ** 2 - 1.0])


def Jg_disk(x):
    return np.array([[2.0 * x[0], 2.0 * x[1]]])


def h_line(x):
    return np.array([x[0] + x[1]])


def Jh_line(x):
    return np.array([[1.0, 1.0]])


def main():
    lower = np.array([-1.0, -1.0])
    upper = np.array([1.0, 1.0])

    nhr_options = nlp_sampling.NHROptions(
        num_samples=50, burn_in=20, random_seed=0, verbose=False,
    )
    restart_options = nlp_sampling.RestartOptions(
        num_restarts=5, strategy="uniform", random_seed=0,
    )
    equality_tol = 1e-3

    with tempfile.TemporaryDirectory() as tmp_dir:
        samples, restart_info, run_dir = nlp_sampling.run_and_report(
            g=g_disk, lower=lower, upper=upper, Jg=Jg_disk,
            h=h_line, Jh=Jh_line,
            nhr_options=nhr_options, restart_options=restart_options,
            joint_names=["x0", "x1"], output_root=tmp_dir,
            note="toy: unit disk cap line", show=False,
            equality_tol=equality_tol, project_samples_to_manifold=True,
            projection_iters=10,
        )

        assert len(samples) > 0, "expected at least one accepted sample"
        assert samples.shape[1] == 2

        assert any(info["status"] == "success" for info in restart_info)

        # Final samples are only guaranteed g <= options.constraint_tol
        # (defaults to good_err_tol) and |h| <= equality_tol - see
        # nhr_sample's is_feasible filter, not a tighter fixed epsilon.
        gvals = np.array([g_disk(x)[0] for x in samples])
        hvals = np.array([h_line(x)[0] for x in samples])
        assert np.all(gvals <= nhr_options.constraint_tol), (
            f"disk inequality violated: max g={gvals.max():.3e}"
        )
        assert np.all(np.abs(hvals) <= equality_tol), (
            f"line equality violated: max |h|={np.abs(hvals).max():.3e}"
        )

        run_dir = Path(run_dir)
        assert run_dir.is_dir()
        assert (run_dir / "info.json").is_file()
        assert (run_dir / "run_analytics_summary.json").is_file()

    print(f"OK: {len(samples)} samples, all within tolerance ({run_dir.name})")


if __name__ == "__main__":
    main()
