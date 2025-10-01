from isaacsim.simulation_app import SimulationApp
experience_path = "/isaac-sim/apps/isaacsim.exp.full.streaming.kit"
simulation_app = SimulationApp(experience=experience_path)

import yaml
import threading
import numpy as np
import omni.usd
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy
from sensor_msgs.msg import PointCloud2
from isaacsim.core.api import World
from pxr import UsdGeom, Gf, Vt

class LidarSubscriber(Node):
    def __init__(self, config):
        super().__init__('lidar_subscriber')
        self.latest_msg = None

        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_ALL)
    
        self.subscription = self.create_subscription(
            PointCloud2,
            config['ros_topic_clean'],
            self.point_cloud_callback,
            qos)
        self.get_logger().info(f"Subscribing to: {config['ros_topic_clean']}")

    def point_cloud_callback(self, msg):
        self.latest_msg = msg

def main():
    with open("src/config.yaml", 'r') as file:
        config = yaml.safe_load(file)

    rclpy.init()
    lidar_node = LidarSubscriber(config)
    spin_thread = threading.Thread(target=rclpy.spin, args=(lidar_node,))
    spin_thread.start()

    try:
        world = World()
        world.scene.add_default_ground_plane()
        stage = omni.usd.get_context().get_stage()
        prim_path = config['prim_path']
        usd_point_cloud = UsdGeom.Points.Define(stage, prim_path)

        world.reset()
        print("--- Digital Twin Controller is running. ---")

        while simulation_app.is_running():
            world.step(render=True)

            if lidar_node.latest_msg:
                msg = lidar_node.latest_msg
                num_fields = msg.point_step // 4
                points_data = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, num_fields)

                if points_data.size > 0:
                    xyz_points = points_data[:, :3]
                    usd_point_cloud.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(xyz_points))
                    usd_point_cloud.GetWidthsAttr().Set(Vt.FloatArray([config['point_size']]))
                    color_primvar = usd_point_cloud.CreateDisplayColorPrimvar()
                    color_primvar.Set([Gf.Vec3f(0.8, 0.1, 0.1)])
                
                lidar_node.latest_msg = None
    finally:
        print("--- Shutting down ---")
        rclpy.shutdown()
        spin_thread.join()
        simulation_app.close()

if __name__ == '__main__':
    main()