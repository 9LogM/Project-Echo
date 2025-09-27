from isaacsim import SimulationApp


experience_path = "/isaac-sim/apps/isaacsim.exp.full.streaming.kit"
simulation_app = SimulationApp(experience=experience_path)

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from isaacsim.core.api import World

print("\n--- Isaac Sim App Initialized with Streaming Experience ---")

try:
    world = World()
    world.scene.add_default_ground_plane()

    world.reset()
    print("--- World has been reset. Simulation is running. ---")

    while simulation_app.is_running():
        world.step(render=True)

finally:
    print("--- Shutting down simulation ---")
    simulation_app.close()