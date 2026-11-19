import pinocchio
from pinocchio import SE3, neutral
from pyhpp.manipulation import Device, urdf, Graph, Problem, ManipulationPlanner
from pyhpp.manipulation.constraint_graph_factory import ConstraintGraphFactory
from pyhpp.constraints import ComparisonType, ComparisonTypes, LockedJoint
from pyhpp.core import RandomShortcut, SimpleTimeParameterization, DiscretizedCollision, Progressive, ProgressiveProjector, Straight, Dichotomy
from pyhpp_viser import Viewer  # noqa: F401
from argparse import ArgumentParser
from pathlib import Path
import numpy as np
import time
import json
SCENE = Path(__file__).resolve().parent / "scene"

# Parse arguments to run experiments from script.

parser = ArgumentParser()
parser.add_argument('-N', default=1, type=int)
parser.add_argument('-D', default=1.0, type=float, help="Radians that the cap should turn. Default is 1.0 rad.")
parser.add_argument('--collision_check', default="continuous", choices=["continuous", "discrete"],
                    help="continuous: Dichotomy. discrete: DiscretizedCollision.")
parser.add_argument('--out_dir', default=None, type=str,
                    help="Folder for the results JSON and .npz. If not given, nothing is saved.")
args = parser.parse_args()

# File paths for the URDF and SRDF files of the robot, hand, cap, and obstacles

arm_urdf_filename = str(SCENE / "panda_arm.urdf")
arm_srdf_filename = str(SCENE / "panda_arm.srdf")
hand_urdf_filename = str(SCENE / "panda_hand.urdf")
hand_srdf_filename = str(SCENE / "panda_hand.srdf")
cap_urdf_filename = str(SCENE / "bottle_cap.urdf")
cap_srdf_filename = str(SCENE / "bottle_cap.srdf")
obstacle_urdf_filename = str(SCENE / "obstacle.urdf")
obstacle_srdf_filename = str(SCENE / "empty.srdf")
floor_urdf_filename = str(SCENE / "floor.urdf")
floor_srdf_filename = str(SCENE / "empty.srdf")

# Create the robot model

robot = Device("1cap_2obs")

# Load the robot, hand, cap, and obstacles into the robot model with their respective poses

urdf.loadModel(robot, 0, "panda_arm", "anchor", arm_urdf_filename, arm_srdf_filename, SE3.Identity())
hand_baseFrame = robot.model().getFrameId("panda_arm/panda_link8") # attach hand to last link of the arm
hand_pose = SE3(pinocchio.rpy.rpyToMatrix(0, 0, -np.pi/4), np.zeros(3))
urdf.loadModel(robot, hand_baseFrame, "panda_hand", "anchor", hand_urdf_filename, hand_srdf_filename, hand_pose)
cap_pose = SE3(pinocchio.rpy.rpyToMatrix(0, 0, 0), np.array([0.5, 0.0, 0.0]))
urdf.loadModel(robot, 0, "bottle_cap", "anchor", cap_urdf_filename, cap_srdf_filename, cap_pose)
obstacle1_pose = SE3(pinocchio.rpy.rpyToMatrix(0, 0, 0), np.array([0.51, 0.031, 0.01]))
urdf.loadModel(robot, 0, "obstacle1", "anchor", obstacle_urdf_filename, obstacle_srdf_filename, obstacle1_pose)
obstacle2_pose = SE3(pinocchio.rpy.rpyToMatrix(0, 0, 0), np.array([0.467, -0.005, 0.01]))
urdf.loadModel(robot, 0, "obstacle2", "anchor", obstacle_urdf_filename, obstacle_srdf_filename, obstacle2_pose)
urdf.loadModel(robot, 0, "floor", "anchor", floor_urdf_filename, floor_srdf_filename, SE3.Identity())

# Create the problem, graph and factory

problem = Problem(robot)
graph = Graph("1cap_2obs", robot, problem)
factory = ConstraintGraphFactory(graph)

### Define hangles, grippers, objetcs

# numerical constraints
graph.maxIterations(50) # These are iterations of the constraint solver, not the planning algorithm.
graph.errorThreshold(1e-4)

# Define grippers
factory.setGrippers(["panda_hand/gripper"])

# Define (movable) objects
objects = ["bottle_cap"]
handlesPerObject = [["bottle_cap/handle"]]
contactsPerObject = [[]]
factory.setObjects(objects, handlesPerObject, contactsPerObject)

# empty environment contacts
environmentContacts = []
factory.environmentContacts(environmentContacts)

factory.generate()

### Define Validation methods of the Problem (e.g. collision checking, joint limits, etc.)

# Collision and joint limit constraints.
problem.addConfigValidation("CollisionValidation") # checks collisions
problem.addConfigValidation("JointBoundValidation") # checks joint limits

# Collision checking: disrete or continuous.

if args.collision_check == "continuous":
    # continuous check with Dichotomy. Tolerance is the safety margin, must be 0.
    discrete_collision_step = None
    problem.pathValidation(Dichotomy(robot, 0.0))
    problem.pathValidationFactory(Dichotomy(robot, 0.0))
else:
    # point check along the path every discrete_collision_step (the HPP default is 0.05).
    discrete_collision_step = 0.005
    problem.pathValidation(DiscretizedCollision(robot, discrete_collision_step))
    problem.pathValidationFactory(DiscretizedCollision(robot, discrete_collision_step))


# Path projector: for each edge in steps of at most 0.2, projects onto the
# constraint manifold, and cuts the edge when discontinuities happen.
problem.steeringMethod(Straight(problem))
problem.pathProjector(ProgressiveProjector(problem.distance(), problem.steeringMethod(), 0.2))

### Add constraints to the graph states and transitions

# The tutorial adds a constraint to keep the gripper open during all transitions, but we don't need it here, only in specific transitions
cts = ComparisonTypes()
cts[:] = [ComparisonType.EqualToZero]   

# Set gripper fingers at grasping position.
graspFingers = [
    LockedJoint(robot, "panda_hand/panda_finger_joint1", np.array([-0.025]), cts),
    LockedJoint(robot, "panda_hand/panda_finger_joint2", np.array([0.025]), cts),
]
# Enforce the gripper finger grasping position only for states "grasp" and "pregrasp"
for name in ["panda_hand/gripper grasps bottle_cap/handle",
             "panda_hand/gripper > bottle_cap/handle | f_pregrasp"]:
    graph.addNumericalConstraintsToState(graph.getState(name), graspFingers)
    
graph.initialize()

# Set weights for sampling along each edge:
weights = {
    "Loop | f": 1,
    "panda_hand/gripper > bottle_cap/handle | f": 2,
    "panda_hand/gripper > bottle_cap/handle | f_ls": 2,
    "Loop | 0-0": 2,
    "panda_hand/gripper < bottle_cap/handle | 0-0": 2,
    "panda_hand/gripper < bottle_cap/handle | 0-0_ls": 2,
}
for name, w in weights.items():
    graph.setWeight(graph.getTransition(name), w)



v = Viewer(robot)
v.initViewer(open=False, loadModel=True)
q = neutral(robot.model())
q[:7] = [0, -np.pi/4, 0, -3*np.pi/4, 0, np.pi/2, np.pi/4]
q[-3:-1] = [-0.025, 0.025]

v(q)

v.setProblem(problem)
v.setGraph(graph)


# Manipulation Planner Problem

q_init = q.copy()
q_goal = q.copy()
q_init[-1] = 3.0
if args.D != 0.0:
    q_goal[-1] = q_init[-1] - args.D
else:
    q_goal[-1] = q_init[-1] - 1.0  # Default to 1 radian if D is set to 0.0
    
print(f"Turning the cap from {q_init[-1]} rad to {q_goal[-1]} rad (delta: {q_init[-1] - q_goal[-1]} rad)")

# Project start and goal onto the "free" state so an invalid query fails loudly here.
res, q_init, err = graph.applyStateConstraints(graph.getState("free"), q_init)
if not res:
    raise RuntimeError(f"Failed to project initial configuration (error {err})")
res, q_goal, err = graph.applyStateConstraints(graph.getState("free"), q_goal)
if not res:
    raise RuntimeError(f"Failed to project goal configuration (error {err})")

problem.initConfig(q_init)
problem.addGoalConfig(q_goal)


problem.constraintGraph(graph)

manipulationPlanner = ManipulationPlanner(problem)
# manipulationPlanner.maxIterations(20000)
timeout_search = 1800  # 30 min, agreed with Florent as a fair benchmark limit
manipulationPlanner.timeOut(timeout_search) 


# Results logged for this run:
results = {
    "scene": "1cap_2obs",
    "collision_check": args.collision_check,
    "discrete_collision_step": discrete_collision_step,
    "D": args.D,
    "cap_init": float(q_init[-1]),
    "cap_goal": float(q_goal[-1]),
    "q_init": q_init.tolist(),
    "q_goal": q_goal.tolist(),
    "number_of_runs(N)": args.N,
    "timeout_search": timeout_search,
    "stop_after_X_runs": 4,
    "stopped_before_N_runs": False,
    "runs": [],
}
paths = {}

# method to save results after each run, in case of crash or timeout
def save_results():
    if args.out_dir is not None:
        # make output directory if it doesn't exist
        out_dir = Path(args.out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        # filename stores collision check method and the cap turn angle D
        filename = f"{args.collision_check}_D{args.D}"
        # write results to JSON 
        with open (out_dir / f"{filename}.json", "w") as f:
            json.dump(results, f, indent=2)
        # write paths to .npz file if any paths were found
        if paths:
            np.savez(out_dir / f"{filename}.npz", **paths) # ** unpacks the dictionary into keyword arguments for np.savez


# Solve the problem and time it.
totalTime = 0.0
totalNumberNodes = 0
success = 0
solutions = list()

for i in range(args.N):
    
    # Stop early if the first X runs all failed.
    if i == results["stop_after_X_runs"] and success == 0:
        print(f"The first {i} runs all failed, stopping.")
        results["stopped_before_N_runs"] = True
        save_results()
        break
    
    # Iteration (run) metadata
    print(f"Run {i+1}/{args.N}")
    run = {"run": i+1, "success": False, "search_time": None, "node_number": None,
           "pathlength_raw": None, "pathlength_shortcut": None, "error": None}
    
    try:
        manipulationPlanner.roadmap().clear()
        problem.resetGoalConfigs()
        problem.initConfig(q_init)
        problem.addGoalConfig(q_goal)
        t0 = time.perf_counter()
        path = manipulationPlanner.solve()
        solutions.append(path)
        t1 = time.perf_counter()
        
    except Exception as e:
        print(f"Run {i+1} failed: {e}")
        run["error"] = str(e)
        results["runs"].append(run)
        save_results()
        
    else:
        success += 1
        totalTime += t1 - t0
        print("Search time: {} s".format(t1 - t0))
        n = len(manipulationPlanner.roadmap().nodes())
        totalNumberNodes += n
        print(f"Number of nodes in the roadmap: {n}")
        
        # Random shortcut optimization
        optimizer = RandomShortcut(problem)
        p1 = optimizer.optimize(path)
        
        # Record this run's numbers and save.
        run.update(success=True, search_time=t1 - t0, node_number=n,
                    pathlength_raw=path.length(), pathlength_shortcut=p1.length())
        results["runs"].append(run)
        
        # Save sampled configurations of both paths.
        for label, p in [("raw", path), ("shortcut", p1)]:
            # sample every 0.01 step along the path.
            t = np.linspace(0.0, p.length(), int(p.length() / 0.01) + 1) 
            # p.eval(ti) returns a tuple (q, success). we keep q.
            paths[f"run{i+1}_{label}"] = np.array([p.eval(ti)[0] for ti in t])
            
        save_results()

        
        # Time parametrization for display
        timeParam = SimpleTimeParameterization(problem)
        timeParam.order = 2
        timeParam.safety = 0.2
        timeParam.maxAcceleration = 1
        p2 = timeParam.optimize(path) # parametrize the original path
        p3 = timeParam.optimize(p1) # parametrize path with shortcuts
        
        # Load the paths to the viewer
        print("Loading paths {}, {}, {}, {} to viewer for run {}".format((i*4)+1, (i*4)+2, (i*4)+3, (i*4)+4, i+1))
        v.loadPath(path)
        v.loadPath(p1)
        v.loadPath(p2)
        v.loadPath(p3)
        

        

if args.N != 0:
    print("#" * 20)
    print(f"Number of rounds: {args.N}")
    print(f"Number of successes: {success}")
    print(f"Success rate: {success / args.N * 100}%")
    if success > 0:
        print(f"Average time per success: {totalTime / success}")
        print(f"Average number nodes per success: {totalNumberNodes / success}")
        
