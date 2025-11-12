#!/usr/bin/env roseus

(load "package://jedy_bringup/euslisp/jedy-interface.l")
(jedy-init)

(send *ri* :servo-on)
(unix::sleep 3)
(send *ri* :angle-vector (send *jedy* :reset-pose) 3000)
(send *ri* :wait-interpolation)
