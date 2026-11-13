#!/usr/bin/env python3
"""Recognise Japanese speech and publish it, with a bubble in front of Jedy.

    ros2 run jedy_speech listener                    # the microphone
    ros2 run jedy_speech listener --wav a.wav b.wav  # files (no microphone needed)

Topics:
    /speech/heard     std_msgs/String   out: what was recognised
    /speech/bubble    visualization_msgs/Marker  out: the "heard" bubble
    /speech/speaking  std_msgs/Bool     in: while Jedy talks, the microphone
                                        is ignored (it would hear itself)
"""

import argparse
import threading

import rclpy
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker

from jedy_speech import (BubbleMaker, Recognizer, load_wav_16k, microphone_utterances,
                         now_s)

#: Light blue, so the person's bubble differs from the robot's white one.
HEARD_STYLE = dict(fill=(215, 235, 255), border=(40, 80, 140), ink=(10, 30, 70))


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--wav', nargs='+', help='recognise these files and exit')
    p.add_argument('--frame', default='base_link')
    p.add_argument('--position', type=float, nargs=3, default=[0.45, -0.35, 0.3],
                   help='where the person bubble floats in --frame [m]')
    p.add_argument('--threshold', type=float, default=0.02,
                   help='microphone level that counts as speech (0..1)')
    args, ros_args = p.parse_known_args()

    rclpy.init(args=ros_args)
    node = rclpy.create_node('jedy_listener')
    heard_pub = node.create_publisher(String, '/speech/heard', 10)
    bubble_pub = node.create_publisher(Marker, '/speech/bubble', 10)
    bubbles = BubbleMaker()
    log = node.get_logger()
    log.info('loading Whisper...')
    recognizer = Recognizer()
    speaking = threading.Event()
    node.create_subscription(
        Bool, '/speech/speaking',
        lambda m: speaking.set() if m.data else speaking.clear(), 10)
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    def publish(audio):
        t0 = now_s()
        text = recognizer.transcribe(audio)
        log.info('heard: "{}" ({:.1f} s of audio, {:.0f} ms)'.format(
            text, len(audio) / 16000, 1000 * (now_s() - t0)))
        if not text:
            return
        heard_pub.publish(String(data=text))
        uri = bubbles.make(text, height_m=0.16, **HEARD_STYLE)
        bubble_pub.publish(bubbles.marker(uri, args.frame, tuple(args.position),
                                          lifetime_s=6.0, ns='heard'))

    try:
        if args.wav:
            import time
            # Wait for whoever listens (voice_command, voice_demo): a fixed
            # second was not enough, discovery takes up to ~3 s here.
            deadline = time.monotonic() + 10.0
            while heard_pub.get_subscription_count() == 0 and time.monotonic() < deadline:
                time.sleep(0.1)
            if heard_pub.get_subscription_count() == 0:
                log.warn('nobody subscribes to /speech/heard; publishing anyway')
            for path in args.wav:
                publish(load_wav_16k(path))
            time.sleep(0.5)
            return
        log.info('listening (Ctrl-C to stop)')
        for audio in microphone_utterances(threshold=args.threshold):
            if speaking.is_set():
                continue
            publish(audio)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
