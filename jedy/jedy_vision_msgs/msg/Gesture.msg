# The hand gesture seen in the image.

# Stamp of the image it was read from.
std_msgs/Header header
# In Japanese: グー, チョキ, パー, 指さし, いいね, よくない, アイラブユー.
# Empty: no hand, or no gesture held steady yet.
string label
# MediaPipe's own name: Closed_Fist, Victory, Open_Palm, Pointing_Up,
# Thumb_Up, Thumb_Down, ILoveYou (empty as above).
string name
# グー, チョキ or パー when the gesture is one of those, else empty.
string janken
# Confidence, 0..1.
float32 score
# Which hand: Left or Right, as MediaPipe reports it.
string handedness
