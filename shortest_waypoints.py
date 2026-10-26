"""The second stage of every planner notebook: the shortest path through the sets the reachability A* chose.

The A*'s placement (Step 6) puts each waypoint at a door point (the Chebyshev centre of two free sets) or at an
extreme wrist point, which is valid but long. shortest_waypoints keeps everything the plan is made of and moves
only the waypoints, to minimise the Euclidean length of the path in (x, alpha): one second-order cone program.

What it keeps, so the plan passes the same check (Step 6b) and has the same grasps:
  - the start and the goal;
  - each move's certifying set: both ends of the move stay in it;
  - each move's kind: a stroke (a move in a grasp set whose wrist goes forward) stays a stroke, with the wrist
    forward and the grasped cap turned by -dwrist; any other move in a grasp set keeps the wrist from going
    forward; and no other move turns a cap.
"""
import numpy as np
from pydrake.solvers import MathematicalProgram, Solve


def shortest_waypoints(waypoints, certifying_set, arm_idx, idx_wrist, idx_caps, grasped_cap, tol=1e-6):
    """waypoints: [(q, label, cert)], cert the set that contains the move into q; certifying_set(cert): that
    set, an HPolyhedron in the cap-free coordinates q[arm_idx]; grasped_cap(cert): the position in idx_caps of
    the cap a grasp set turns. Returns the moved waypoints, or the given ones if the solver fails."""
    Q0 = np.array([q for q, _, _ in waypoints], dtype=float)
    n, nq = Q0.shape
    arm_idx, idx_caps = np.asarray(arm_idx), np.asarray(idx_caps)
    prog = MathematicalProgram()
    Q = [prog.NewContinuousVariables(nq, f"q{i}") for i in range(n)]
    for i in (0, n - 1):                                     # the start and the goal stay where they are
        prog.AddBoundingBoxConstraint(Q0[i], Q0[i], Q[i])
    for i in range(n):
        prog.SetInitialGuess(Q[i], Q0[i])
    for i in range(1, n):
        cert = waypoints[i][2]
        P = certifying_set(cert)
        for j in (i - 1, i):
            if 0 < j < n - 1:                                # the fixed ends are in their sets already
                prog.AddLinearConstraint(P.A(), np.full(len(P.b()), -np.inf), P.b(), Q[j][arm_idx])
        dq = Q[i] - Q[i - 1]
        dw, dalpha = dq[idx_wrist], dq[idx_caps]
        turned = None
        if cert[0] == "G":
            if Q0[i, idx_wrist] - Q0[i - 1, idx_wrist] > tol:   # a stroke: the grasped cap turns by -dwrist
                turned = grasped_cap(cert)
                prog.AddLinearConstraint(dw >= 0)
                prog.AddLinearConstraint(dalpha[turned] + dw == 0)
            else:
                prog.AddLinearConstraint(dw <= 0)
        for k in range(len(idx_caps)):
            if k != turned:
                prog.AddLinearConstraint(dalpha[k] == 0)
        # length of the move: |q_i - q_{i-1}|, as a cone
        prog.AddL2NormCostUsingConicConstraint(np.hstack([np.eye(nq), -np.eye(nq)]), np.zeros(nq),
                                               np.concatenate([Q[i], Q[i - 1]]))
    result = Solve(prog)
    if not result.is_success():
        print(f"shortest_waypoints: {result.get_solution_result()}, keeping the placed waypoints")
        return waypoints
    return [(result.GetSolution(Q[i]), label, cert) for i, (_, label, cert) in enumerate(waypoints)]
