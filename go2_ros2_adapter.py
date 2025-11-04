from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from omni.isaac.core import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import SingleArticulation as Articulation
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from pxr import Gf
from isaaclab_assets.robots.unitree import UNITREE_GO2_CFG
from isaaclab.sensors import RayCasterCfg, patterns, ContactSensorCfg
import isaaclab

# 初始化 ROS2
rclpy.init(args=None)


class Go2ROS2Bridge(Node):
    """Isaac Sim 与 ROS2 的通信桥"""

    def __init__(self, robot: Articulation):
        super().__init__("go2_ros2_bridge")
        self.robot = robot

        # ROS2 发布与订阅
        self.odom_pub = self.create_publisher(Odometry, "/unitree_go2/odom", 10)
        self.cmd_sub = self.create_subscription(
            Twist, "/unitree_go2/cmd_vel", self.cmd_vel_callback, 10
        )

        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_wz = 0.0

    def cmd_vel_callback(self, msg: Twist):
        self.cmd_vx = msg.linear.x
        self.cmd_vy = msg.linear.y
        self.cmd_wz = msg.angular.z
        self.get_logger().info(
            f"接收到 cmd_vel -> vx:{self.cmd_vx:.2f}, vy:{self.cmd_vy:.2f}, wz:{self.cmd_wz:.2f}"
        )

    def publish_odom(self):
        pose, _ = self.robot.get_world_pose()
        lin_vel = self.robot.get_linear_velocity()
        ang_vel = self.robot.get_angular_velocity()

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"

        msg.pose.pose.position.x = float(pose[0])
        msg.pose.pose.position.y = float(pose[1])
        msg.pose.pose.position.z = float(pose[2])
        msg.pose.pose.orientation.w = 1.0

        msg.twist.twist.linear.x = float(lin_vel[0])
        msg.twist.twist.linear.y = float(lin_vel[1])
        msg.twist.twist.linear.z = float(lin_vel[2])
        msg.twist.twist.angular.z = float(ang_vel[2])

        self.odom_pub.publish(msg)

    def apply_velocity(self):
        vel = Gf.Vec3d(self.cmd_vx, self.cmd_vy, 0.0)
        self.robot.set_linear_velocity(vel)


# === 初始化仿真世界 ===
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# === 加载 Unitree Go2 模型 ===

from isaacsim.core.utils.nucleus import get_assets_root_path

go2_prim_path = "/World/Go2"

# 尝试使用 Nucleus 服务器路径
nucleus_path = get_assets_root_path()
if nucleus_path:
    go2_usd_path = f"{nucleus_path}/Isaac/Robots/Unitree/Go2/go2.usd"
else:
    # 使用本地 Isaac Lab 路径作为备选
    isaaclab_path = os.environ.get("ISAACLAB_PATH", "/home/wll/IsaacLab")
    go2_usd_path = f"{isaaclab_path}/source/isaaclab_assets/data/Robots/Unitree/Go2/go2.usd"
    if not os.path.exists(go2_usd_path):
        go2_usd_path = f"{isaaclab_path}/Isaac/IsaacLab/Robots/Unitree/Go2/go2.usd"

# 将机器人 USD 添加到场景
add_reference_to_stage(usd_path=go2_usd_path, prim_path=go2_prim_path)

# 创建机器人 Articulation 实例
unitree_go2 = world.scene.add(Articulation(prim_path=go2_prim_path, name="Go2"))

world.reset()
bridge = Go2ROS2Bridge(unitree_go2)

print("=" * 50)
print("Unitree Go2 ROS2 Bridge 已启动")
print("订阅话题: /unitree_go2/cmd_vel")
print("发布话题: /unitree_go2/odom")
print("=" * 50)

# === 主循环 ===
try:
    print("进入主循环...")
    while simulation_app.is_running():
        # ROS2 处理
        rclpy.spin_once(bridge, timeout_sec=0.001)
        
        # 应用速度命令
        bridge.apply_velocity()
        
        # 发布里程计
        bridge.publish_odom()
        
        # 推进仿真
        world.step(render=True)
        
except KeyboardInterrupt:
    print("\n收到键盘中断信号，正在关闭...")
except Exception as e:
    print(f"\n发生错误: {e}")
    import traceback
    traceback.print_exc()
finally:
    print("清理资源...")
    bridge.destroy_node()
    rclpy.shutdown()
    simulation_app.close()
    print("程序已退出")
