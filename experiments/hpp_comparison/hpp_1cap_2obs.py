import pinocchio
from pinocchio import SE3, neutral
from pyhpp.manipulation import Device, urdf, Graph, Problem, ManipulationPlanner
from pyhpp.manipulation.constraint_graph_factory import ConstraintGraphFactory
from pyhpp.constraints import ComparisonType, ComparisonTypes, LockedJoint
from pyhpp_viser import Viewer  # noqa: F401
from pathlib import Path
import numpy as np
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
urdf.loadModel(robot, hand_baseFrame, "panda_hand", "anchor", hand_urdf_filename, hand_srdf_filename, SE3(pinocchio.rpy.rpyToMatrix(0, 0, -np.pi/4), np.zeros(3)))
urdf.loadModel(robot, 0, "bottle_cap", "anchor", cap_urdf_filename, cap_srdf_filename, SE3(np.eye(3), np.array([0.5, 0.0, 0.0])))
urdf.loadModel(robot, 0, "obstacle", "anchor", obstacle_urdf_filename, obstacle_srdf_filename, SE3.Identity())



problem = Problem(robot)
graph = Graph("1cap_2obs", robot, problem)
factory = ConstraintGraphFactory(graph)

# numerical constraints
graph.maxIterations(1000)
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

# The tutorial adds a constraint to keep the gripper open during all transitions, but we don't need it here (I think?)
cts = ComparisonTypes()
cts[:] = [ComparisonType.EqualToZero]   

# Set gripper fingers at grasping position.
graspFingers = [
    LockedJoint(robot, "panda_hand/panda_finger_joint1", np.array([-0.025]), cts),
    LockedJoint(robot, "panda_hand/panda_finger_joint2", np.array([0.025]), cts),
]
# Enforce the gripper finger grasping position only for states "grasp", and "pregrasp"
for name in ["panda_hand/gripper grasps bottle_cap/handle",
             "panda_hand/gripper > bottle_cap/handle | f_pregrasp"]:
    graph.addNumericalConstraintsToState(graph.getState(name), graspFingers)
    
graph.initialize()


v = Viewer(robot)
v.initViewer(open=False, loadModel=True)
q = neutral(robot.model())
q[-3:-1] = [-0.025, 0.025]
v(q)

v.setProblem(problem)
v.setGraph(graph)