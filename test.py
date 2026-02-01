from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.articulations import Articulation
from pxr import Gf
from isaaclab_assets.robots.unitree import UNITREE_GO2_CFG

print(UNITREE_GO2_CFG)