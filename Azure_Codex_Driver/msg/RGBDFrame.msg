# Both images come from one SDK capture and use the color optical frame.
# The observation header uses the color exposure time. Each image keeps its own stamp.
std_msgs/Header header
sensor_msgs/Image rgb
# Registered to the RGB pixel grid, 32FC1 metres, zero where depth is unavailable.
sensor_msgs/Image depth
sensor_msgs/CameraInfo camera_info
