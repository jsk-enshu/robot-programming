#!/usr/bin/env python3
"""Start and stop the demos by voice.

    ros2 launch jedy_bringup jedy_gazebo.launch.py demo:=true person:=true camera:=d405
    ros2 launch voicevox voicevox_texttospeech.launch.xml     # speaker
    ros2 launch ros_speech_recognition speech_recognition.launch.xml language:=ja-JP  # listener
    ros2 run jedy_vision voice_demo

What it understands (keywords; read ``COMMANDS`` and add your own):
    コップを探して / いすを探して / 人を探して   -> find_object cup / chair / person
    ついてきて                                  -> follow_person --keep
    こっちを見て                                -> look_at_person --keep
    まねして                                    -> mimic_arms
    箱に近づいて / タッチして / 持って           -> approach_tag / touch_tag / grasp_tag
    手を振って                                  -> wave_back
    止まって                                    -> stop the running demo

One demo runs at a time, as a separate ``ros2 run`` process (exactly what
you would type), stopped with Ctrl-C (SIGINT) like you would.
"""

import argparse
import os
import re
import signal
import subprocess
import unicodedata

import rclpy
from speech_recognition_msgs.msg import SpeechRecognitionCandidates

from jedy_vision import say

#: (pattern, reply, command). First match wins.
COMMANDS = [
    (r'(コップ|カップ).*(探|さが)', 'コップを探すのだ', ['find_object', 'cup']),
    (r'(いす|椅子|イス).*(探|さが)', 'いすを探すのだ', ['find_object', 'chair']),
    (r'人.*(探|さが)', '人を探すのだ', ['find_object', 'person']),
    (r'(ついて|付いて)(き|来)', 'ついていくのだ', ['follow_person', '--keep']),
    (r'(こっち|私|わたし).*(見|み)', '見ているのだ', ['look_at_person', '--keep']),
    (r'(まね|真似|マネ)', 'まねするのだ', ['mimic_arms']),
    (r'(箱|はこ|タグ).*(近づ|ちかづ|行って)', '箱に近づくのだ', ['approach_tag']),
    (r'(タッチ|さわって|触って)', 'タッチするのだ', ['touch_tag']),
    (r'(持って|もって|つかんで|掴んで)', '箱を持つのだ', ['grasp_tag']),
    (r'手を(振|ふ)', '手を振るのだ', ['wave_back', '--test']),
]
STOP = r'(止ま|とま|ストップ|やめ)'


def normalize(text):
    text = unicodedata.normalize('NFKC', text)
    return re.sub(r'[\s、。,.!?！？]', '', text)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    # Leave ROS's own arguments (``--ros-args -r ...``) to rclpy.
    p.parse_args(rclpy.utilities.remove_ros_args()[1:])

    rclpy.init()
    node = rclpy.create_node('jedy_voice_demo')
    running = {'proc': None, 'name': None}
    log = node.get_logger()

    def stop():
        """Ctrl-C the running demo; True once it has really exited."""
        proc = running['proc']
        if proc is None or proc.poll() is not None:
            running['proc'] = None
            return True
        # The whole process group, like Ctrl-C in its terminal: `ros2 run`
        # does not pass SIGINT on to the program it started, and killing
        # only `ros2 run` left the demo running on its own.
        group = os.getpgid(proc.pid)
        os.killpg(group, signal.SIGINT)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(group, signal.SIGKILL)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                log.error('{} (pid {}) did not exit'.format(running['name'], proc.pid))
                return False
        running['proc'] = None
        return True

    def on_heard(msg):
        if not msg.transcript:
            return
        heard = msg.transcript[0]
        text = normalize(heard)
        log.info('heard: ' + heard)
        if re.search(STOP, text):
            if running['proc'] is None:
                say(node, '何も動いていないのだ')
            elif stop():
                say(node, '止まったのだ')
            else:
                say(node, '止められないのだ。Ctrl-C で止めてほしいのだ')
            return
        for pattern, reply, command in COMMANDS:
            if re.search(pattern, text):
                if not stop():
                    say(node, '前のデモが止まらないのだ')
                    return
                say(node, reply)
                cmd = ['ros2', 'run', 'jedy_vision'] + command
                log.info('run: ' + ' '.join(cmd))
                running['proc'] = subprocess.Popen(cmd, start_new_session=True)
                running['name'] = command[0]
                return
        say(node, 'すみません、もう一度言ってください')

    node.create_subscription(SpeechRecognitionCandidates, '/speech_to_text', on_heard, 10)
    log.info('waiting for /speech_to_text (say "コップを探して")')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop()


if __name__ == '__main__':
    main()
