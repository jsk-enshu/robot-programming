#!/usr/bin/env python3
"""Make Jedy say something (needs `ros2 run jedy_speech speaker` running).

    ros2 run jedy_speech say "こんにちは、ジェディです"
    ros2 run jedy_speech say --wav out.wav "テスト"   # also save the speech to a file

The same as ``ros2 topic pub --once /speech/say std_msgs/msg/String "{data: ...}"``,
but waits until the speaker is listening, so the first message is not lost.
"""

import argparse
import time

import rclpy
from std_msgs.msg import String


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('text')
    p.add_argument('--wav', help='also write the speech to this WAV file (no ROS needed)')
    p.add_argument('--timeout', type=float, default=10.0)
    p.add_argument('--engine', choices=['openjtalk', 'voicevox'], default='openjtalk',
                   help='for --wav only (the speaker node has its own --engine)')
    p.add_argument('--voice', default='ずんだもん')
    args, ros_args = p.parse_known_args()

    if args.wav:
        from jedy_speech import Speaker, VoicevoxSpeaker, save_wav
        speaker = VoicevoxSpeaker(args.voice) if args.engine == 'voicevox' else Speaker()
        samples, rate = speaker.synthesize(args.text)
        if speaker.credit:
            print('credit required wherever this audio is used:', speaker.credit)
        save_wav(args.wav, samples, rate)
        print('saved', args.wav)
        return

    rclpy.init(args=ros_args)
    node = rclpy.create_node('jedy_say')
    pub = node.create_publisher(String, '/speech/say', 10)
    deadline = time.monotonic() + args.timeout
    while pub.get_subscription_count() == 0:
        if time.monotonic() > deadline:
            raise SystemExit('nobody listens on /speech/say: start '
                             '`ros2 run jedy_speech speaker` first')
        rclpy.spin_once(node, timeout_sec=0.1)
    pub.publish(String(data=args.text))
    rclpy.spin_once(node, timeout_sec=0.3)


if __name__ == '__main__':
    main()
