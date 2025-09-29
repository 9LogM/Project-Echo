from isaacsim.simulation_app import SimulationApp
experience_path = "/isaac-sim/apps/isaacsim.exp.full.streaming.kit"
simulation_app = SimulationApp(experience=experience_path)

import yaml
import numpy as np
import struct
import omni.usd
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from isaacsim.core.api import World
from pxr import UsdGeom, Gf, Vt

def pointcloud2_to_numpy(msg: PointCloud2):
    offset_x, offset_y, offset_z = -1, -1, -1
    for field in msg.fields:
        if field.name == 'x': offset_x = field.offset
        elif field.name == 'y': offset_y = field.offset
        elif field.name == 'z': offset_z = field.offset
    if any(offset < 0 for offset in [offset_x, offset_y, offset_z]):
        return np.array([])
    point_step = msg.point_step
    num_points = msg.width * msg.height
    points = np.zeros((num_points, 3), dtype=np.float32)
    data = msg.data
    for i in range(num_points):
        base_idx = i * point_step
        points[i, 0] = struct.unpack_from('f', data, base_idx + offset_x)[0]
        points[i, 1] = struct.unpack_from('f', data, base_idx + offset_y)[0]
        points[i, 2] = struct.unpack_from('f', data, base_idx + offset_z)[0]
    return points

class LidarSubscriber(Node):
    def __init__(self, config):
        super().__init__('lidar_subscriber')
        self.config = config
        self.latest_points = None
        self.subscription = self.create_subscription(
            PointCloud2,
            self.config['ros_topic'],
            self.point_cloud_callback,
            10)
        self.get_logger().info(f"Subscribing to topic: {self.config['ros_topic']}")
    def point_cloud_callback(self, msg):
        self.latest_points = pointcloud2_to_numpy(msg)

def main():
    with open("src/config.yaml", 'r') as file:
        config = yaml.safe_load(file)

    rclpy.init()
    lidar_node = LidarSubscriber(config)

    try:
        world = World()
        world.scene.add_default_ground_plane()
        
        stage = omni.usd.get_context().get_stage()
        prim_path = config['prim_path']

        usd_point_cloud = UsdGeom.Points.Define(stage, prim_path)
        xformable = UsdGeom.Xformable(usd_point_cloud.GetPrim())
        translate_op = xformable.AddTranslateOp()
        translate_op.Set(Gf.Vec3d(0, 0, 0.5))

        world.reset()
        print("--- World has been reset. Simulation is running. ---")

        while simulation_app.is_running():
            world.step(render=True)
            rclpy.spin_once(lidar_node, timeout_sec=0.0)

            if lidar_node.latest_points is not None:
                points_data = lidar_node.latest_points
                if points_data.size > 0:
                    usd_point_cloud.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(points_data))

                    usd_point_cloud.GetWidthsAttr().Set(Vt.FloatArray([config['point_size']]))

                    color_primvar = usd_point_cloud.CreateDisplayColorPrimvar()
                    color_primvar.Set([Gf.Vec3f(0.8, 0.1, 0.1)])
        
                lidar_node.latest_points = None

    finally:
        print("--- Shutting down ---")
        rclpy.shutdown()
        simulation_app.close()

if __name__ == '__main__':
    main()