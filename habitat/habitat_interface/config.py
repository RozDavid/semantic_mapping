

output_directory = "output/"

rgb_sensor = True
depth_sensor = True
semantic_sensor = True

sim_settings = {
    "width": 640, #1280,  # Spatial resolution of the observations
    "height": 480, #720,
    "default_agent": 0,
    "sensor_height": 1.0,  # Height of sensors in meters
    "color_sensor": rgb_sensor,  # RGB sensor
    "depth_sensor": depth_sensor,  # Depth sensor
    "semantic_sensor": semantic_sensor,  # Semantic sensor
    "seed": 0,  # used in the random navigation
    "enable_physics": False,  # kinematics only
    "depth_offset": 0.0 # Offset of the depth sensor on the horizontal axis in meters
}
