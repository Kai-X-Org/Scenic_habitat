import habitat_sim
import numpy as np
test_scene = '/home/ek65/habitat-lab/data/scene_datasets/habitat-test-scenes/apartment_1.glb'
sim_settings = {
    "scene": test_scene,  # Scene path
    "default_agent": 0,  # Index of the default agent
    "sensor_height": 1.5,  # Height of sensors in meters, relative to the agent
    "width": 256,  # Spatial resolution of the observations
    "height": 256,
}

def make_simple_cfg(settings):
    # simulator backend
    sim_cfg = habitat_sim.SimulatorConfiguration()
    sim_cfg.scene_id = settings["scene"]

    # agent
    agent_cfg = habitat_sim.agent.AgentConfiguration()

    # In the 1st example, we attach only one sensor,
    # a RGB visual sensor, to the agent
    # rgb_sensor_spec = habitat_sim.SensorSpec()
    # rgb_sensor_spec.uuid = "color_sensor"
    # rgb_sensor_spec.sensor_type = habitat_sim.SensorType.COLOR
    # rgb_sensor_spec.resolution = [settings["height"], settings["width"]]
    # rgb_sensor_spec.position = [0.0, settings["sensor_height"], 0.0]

    # agent_cfg.sensor_specifications = [rgb_sensor_spec]

    return habitat_sim.Configuration(sim_cfg, [agent_cfg])

cfg = make_simple_cfg(sim_settings)



print('got here')
try:  # Needed to handle out of order cell run in Colab
    sim.close()
except NameError:
    pass
print('got here 2')
sim = habitat_sim.Simulator(cfg)
print('got here 3')


agent = sim.initialize_agent(sim_settings["default_agent"])

# Set agent state
agent_state = habitat_sim.AgentState()
agent_state.position = np.array([-0.6, 0.0, 0.0])  # in world space
agent.set_state(agent_state)

# Get agent state
agent_state = agent.get_state()
print("agent_state: position", agent_state.position, "rotation", agent_state.rotation)
action_names = list(cfg.agents[sim_settings["default_agent"]].action_space.keys())
print("Discrete action space: ", action_names)
