import pinocchio
from pinocchio import SE3, neutral
from pyhpp.manipulation import Device, urdf
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
urdf.loadModel(robot, 0, "bottle_cap", "anchor", cap_urdf_filename, cap_srdf_filename, SE3.Identity())
urdf.loadModel(robot, 0, "obstacle", "anchor", obstacle_urdf_filename, obstacle_srdf_filename, SE3.Identity())





v = Viewer(robot)
v.initViewer(open=False, loadModel=True)
q = neutral(robot.model())
q[-3:-1] = [-0.025, 0.025]
v(q)