# One object found in the image.

# The word it was asked for, as asked (e.g. "ペットボトル" or "bottle").
string label
# Confidence, 0..1.
float32 score
# Box in the image [px]: x1, y1 (top left), x2, y2 (bottom right).
float32[4] box
# Where it is [m], in the image's frame (header.frame_id of the
# DetectionArray: the camera's optical frame, x right, y down, z forward):
# the box centre's pixel at the median depth of the box's middle third.
# has_position is false when there is no depth image, it does not match the
# colour image's size, or no pixel in that part has a depth reading (too
# near or too far for the camera; the D405 measures 0.07-0.5 m).
geometry_msgs/Point position
bool has_position
