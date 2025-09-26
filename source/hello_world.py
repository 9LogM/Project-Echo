from omni.isaac.kit import SimulationApp

CONFIG = {
    "experience": "/isaac-sim/apps/isaacsim.exp.full.streaming.kit"
}

simulation_app = SimulationApp(CONFIG)

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicSphere

print("\n--- Isaac Sim App Initialized with Streaming Experience ---")

world = World()

world.scene.add_default_ground_plane()
DynamicSphere(
    prim_path="/World/DroppingBall",
    name="my_ball",
    position=np.array([0, 0, 2.0]),
    radius=0.25,
    color=np.array([0, 0, 1.0]),
)

world.reset()
print("--- World has been reset. Simulation is running. ---")

while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()