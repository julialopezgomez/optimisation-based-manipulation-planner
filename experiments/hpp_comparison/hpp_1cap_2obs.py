import pinocchio
from pinocchio import SE3, neutral
from pyhpp.manipulation import Device, urdf, Graph, Problem, ManipulationPlanner
from pyhpp.manipulation.constraint_graph_factory import ConstraintGraphFactory
from pyhpp.constraints import ComparisonType, ComparisonTypes, LockedJoint
from pyhpp.core import RandomShortcut, SimpleTimeParameterization, DiscretizedCollision, Progressive
from pyhpp_viser import Viewer  # noqa: F401
from pathlib import Path
import numpy as np
import time
SCENE = Path(__file__).resolve().parent / "scene"


robot = Device("1cap_2obs")

arm_urdf_filename = str(SCENE / "panda_arm.urdf")
arm_srdf_filename = str(SCENE / "panda_arm.srdf")
hand_urdf_filename = str(SCENE / "panda_hand.urdf")
hand_srdf_filename = str(SCENE / "panda_hand.srdf")
cap_urdf_filename = str(SCENE / "bottle_cap.urdf")
cap_srdf_filename = str(SCENE / "bottle_cap.srdf")
obstacle_urdf_filename = str(SCENE / "obstacle.urdf")
obstacle_srdf_filename = str(SCENE / "empty.srdf")


urdf.loadModel(robot, 0, "panda_arm", "anchor", arm_urdf_filename, arm_srdf_filename, SE3.Identity())
hand_baseFrame = robot.model().getFrameId("panda_arm/panda_link8")
hand_pose = SE3(pinocchio.rpy.rpyToMatrix(0, 0, -np.pi/4), np.zeros(3))
urdf.loadModel(robot, hand_baseFrame, "panda_hand", "anchor", hand_urdf_filename, hand_srdf_filename, hand_pose)
cap_pose = SE3(pinocchio.rpy.rpyToMatrix(0, 0, 0), np.array([0.5, 0.0, 0.0]))
urdf.loadModel(robot, 0, "bottle_cap", "anchor", cap_urdf_filename, cap_srdf_filename, cap_pose)
obstacle1_pose = SE3(pinocchio.rpy.rpyToMatrix(0, 0, 0), np.array([0.51, 0.031, 0.01]))
urdf.loadModel(robot, 0, "obstacle1", "anchor", obstacle_urdf_filename, obstacle_srdf_filename, obstacle1_pose)
obstacle2_pose = SE3(pinocchio.rpy.rpyToMatrix(0, 0, 0), np.array([0.467, -0.005, 0.01]))
urdf.loadModel(robot, 0, "obstacle2", "anchor", obstacle_urdf_filename, obstacle_srdf_filename, obstacle2_pose)


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
problem.addConfigValidation("CollisionValidation")
problem.addConfigValidation("JointBoundValidation")

# (a) point check with a finer step (the default is 0.05)
problem.pathValidation(DiscretizedCollision(robot, 0.001))
problem.pathValidationFactory(DiscretizedCollision(robot, 0.001))

# # (b) continuous check over the whole path
# problem.pathValidation(Progressive(robot, 0.001))
# problem.pathValidationFactory(Progressive(robot, 0.001))


### Add constraints to the graph states and transitions

# The tutorial adds a constraint to keep the gripper open during all transitions, but we don't need it here (I think?)
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
q_init[-1] = 1.0
q_goal[-1] = -1.0

problem.initConfig(q_init)
problem.addGoalConfig(q_goal)


problem.constraintGraph(graph)

manipulationPlanner = ManipulationPlanner(problem)
# manipulationPlanner.maxIterations(20000)
manipulationPlanner.timeOut(1200)  

# Timed solution search
t0 = time.perf_counter()
try:
    p = manipulationPlanner.solve()
finally:
    print(f"Search time: {time.perf_counter() - t0:.2f} s")


# Optimizer:
optimizer = RandomShortcut(problem)
p1 = optimizer.optimize(p)

# Time parametrization:
timeParam = SimpleTimeParameterization(problem)
timeParam.order = 2
timeParam.safety = 0.2
timeParam.maxAcceleration = 1
p2 = timeParam.optimize(p1)
p3 = timeParam.optimize(p)


# Load the path to the viewer so that we can display it.
v.loadPath(p)
v.loadPath(p1)
v.loadPath(p2)
v.loadPath(p3)