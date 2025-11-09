#!/usr/bin/env roseus

(load "package://jedy_bringup/euslisp/jedy.l")
(setq *jedy* (jedy))
(load "models/arrow-object.l")
(setq *rarm-end-coords-axes* (arrow))
(setq *larm-end-coords-axes* (arrow))
(setq *head-end-coords-axes* (arrow))
(objects (list *jedy* *rarm-end-coords-axes* *larm-end-coords-axes* *head-end-coords-axes*))
(send *rarm-end-coords-axes* :newcoords (send (send *jedy* :rarm-end-coords) :copy-worldcoords))
(send *larm-end-coords-axes* :newcoords (send (send *jedy* :larm-end-coords) :copy-worldcoords))
(send *head-end-coords-axes* :newcoords (send (send *jedy* :head-end-coords) :copy-worldcoords))
(send *irtviewer* :draw-objects)
