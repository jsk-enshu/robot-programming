# Motor sounds for `jedy_sim_sounds`

Recorded from the real Jedy (RCB-4mini) with a phone, 2026-11-12, and cut into
4 s loops whose end is crossfaded (0.3 s) into their start.

| file | recording | what |
| --- | --- | --- |
| `wheels.wav` | robot held off the floor, `ros2 run jedy_skrobot drive_steady` | the four wheel servos at 0.12 m/s; 2.0-6.0 s of the clip, RMS -18 dBFS |
| `servos.wav` | `ros2 run jedy_skrobot swing_joint` (head, +-30 deg, 0.4 s each way) | the head servo; 5.0-9.0 s of the clip, RMS -25.6 dBFS (pulsed: one burst per swing) |
