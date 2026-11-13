"""Which axis and button is what, in ONE place.

Imported by both the teleop node and the monitor, so what the monitor shows you
is by construction what the node acts on. Two copies of this would be two
copies that can disagree, and the way you would find out is a button doing
something other than its label -- which on a robot that can drive away is the
worst kind of wrong.

The numbers are joydev's, as `joy_linux` publishes them. They are NOT
interchangeable with SDL's: the `joy` package reorders some pads, so a mapping
verified under one driver says nothing about the other. Check any pad with

    ros2 run jedy_teleop jedy_joy_monitor

which shows every axis and button live, against what is written here.
"""

AXIS_LEFT_X = 0
AXIS_LEFT_Y = 1
AXIS_RIGHT_X = 3
AXIS_DPAD_X = 6
AXIS_DPAD_Y = 7

BUTTON_CROSS = 0
BUTTON_CIRCLE = 1
BUTTON_TRIANGLE = 2
BUTTON_SQUARE = 3
BUTTON_L1 = 4
BUTTON_R1 = 5
# joydev names the buttons in index order, which `jstest /dev/input/js0` shows:
# BtnA BtnB BtnX BtnY BtnTL BtnTR BtnTL2 BtnTR2 ... -- so BtnTL2, the L2
# trigger read as a switch, is 6. L2 is also an analogue axis (2, "Z"); the
# teleop node uses the button, because a trigger that has not been touched
# since the pad connected reads 0.0 rather than "released" and a speed control
# must not start life at half.
BUTTON_L2 = 6

#: What each index does, for display. Anything absent is unused by the teleop
#: node, and the monitor says so rather than leaving a blank an operator has to
#: interpret.
AXIS_ROLES = {
    AXIS_LEFT_X: ("left stick X", "strafe  (+ = left)"),
    AXIS_LEFT_Y: ("left stick Y", "drive   (+ = forward)"),
    AXIS_RIGHT_X: ("right stick X", "turn    (+ = left)"),
    AXIS_DPAD_X: ("D-pad X", "head pan"),
    AXIS_DPAD_Y: ("D-pad Y", "head tilt"),
}

BUTTON_ROLES = {
    BUTTON_CROSS: ("cross", "with L1+R1 held: go to the HOME pose"),
    BUTTON_CIRCLE: ("circle", "RELEASE torque -- no deadman needed"),
    BUTTON_TRIANGLE: ("triangle", "torque ON (re-syncs controllers first)"),
    BUTTON_L1: ("L1", "DEADMAN -- nothing moves without it"),
    BUTTON_R1: ("R1", "slow mode"),
    BUTTON_L2: ("L2", "head twice as fast (the base has no faster)"),
}
