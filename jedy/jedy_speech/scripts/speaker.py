#!/usr/bin/env python3
"""Say what arrives on /speech/say, with a speech bubble above Jedy in RViz.

    ros2 run jedy_speech speaker              # terminal 1 (or speech_demo.launch.py)
    ros2 run jedy_speech say "こんにちは"      # terminal 2

Topics:
    /speech/say       std_msgs/String   in: text to say
    /speech/bubble    visualization_msgs/Marker  out: the bubble (RViz)
    /speech/speaking  std_msgs/Bool     out: True while talking (listener
                                        ignores the microphone meanwhile)

Voices: ``--engine openjtalk`` (default) or ``--engine voicevox --voice ずんだもん``
(after ``ros2 run jedy_speech setup_voicevox``; the bubble then carries the
"VOICEVOX:ずんだもん" credit its terms require).

``--no-audio`` only shows the bubble (a PC without speakers). Without it, a
missing or failing sound device is reported on every utterance, not hidden.
"""

import argparse

import rclpy
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker

from jedy_speech import BubbleMaker, Speaker, VoicevoxSpeaker, now_s


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--frame', default='base_link', help='frame the bubble hangs from')
    p.add_argument('--height', type=float, default=0.55,
                   help='bubble bottom above the frame [m] (Jedy head is ~0.4 m)')
    p.add_argument('--no-audio', action='store_true', help='bubble only, no sound')
    p.add_argument('--engine', choices=['openjtalk', 'voicevox'], default='openjtalk')
    p.add_argument('--voice', default='ずんだもん', help='VOICEVOX character')
    p.add_argument('--style', default='ノーマル', help='VOICEVOX style')
    args, ros_args = p.parse_known_args()

    rclpy.init(args=ros_args)
    node = rclpy.create_node('jedy_speaker')
    bubble_pub = node.create_publisher(Marker, '/speech/bubble', 10)
    speaking_pub = node.create_publisher(Bool, '/speech/speaking', 10)
    bubbles = BubbleMaker()
    log = node.get_logger()
    if args.engine == 'voicevox':
        speaker = VoicevoxSpeaker(args.voice, args.style)
        log.info('voice: {} ({})'.format(speaker.credit, args.style))
    else:
        speaker = Speaker()

    def on_say(msg):
        text = msg.data.strip()
        if not text:
            return
        t0 = now_s()
        samples, rate = speaker.synthesize(text)
        duration = len(samples) / rate
        uri = bubbles.make(text, credit=speaker.credit)
        bubble_pub.publish(bubbles.marker(uri, args.frame, (0.0, 0.0, args.height),
                                          lifetime_s=duration + 2.0, ns='said'))
        log.info('say: {} ({:.1f} s of speech, ready in {:.0f} ms)'.format(
            text, duration, 1000 * (now_s() - t0)))
        speaking_pub.publish(Bool(data=True))
        try:
            if not args.no_audio:
                Speaker.play(samples, rate)
        except RuntimeError as exc:
            log.error('no sound: {} (bubble only; --no-audio to silence this)'.format(exc))
        finally:
            speaking_pub.publish(Bool(data=False))

    node.create_subscription(String, '/speech/say', on_say, 10)
    log.info('ready: publish text on /speech/say')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
