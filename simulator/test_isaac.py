from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

from isaacsim.core.api import World

world = World()
world.reset()

for i in range(5):
    world.step(render=False)
    print(f"Step {i+1} OK")

print("Isaac Sim headless test passed.")
simulation_app.close()
