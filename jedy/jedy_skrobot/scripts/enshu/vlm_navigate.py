#!/usr/bin/env python3
"""Drive the base towards a goal given in words, deciding with a VLM.

The 2026 version of jedy_bringup/scripts/robocrew_image_to_cmd_vel.py
(2025: langchain, Google AI Studio and a GOOGLE_API_KEY). Here the camera
image and the goal go to the course's VLM gateway (ri.ask) with the AI TA's key
(``ta login``), and the model answers with one small move::

    ros2 run jedy_skrobot vlm_navigate 赤い箱に近づいて
    ros2 run jedy_skrobot vlm_navigate               # wait for a goal on the topic
    ros2 topic pub --once /vlm_nav/prompt std_msgs/msg/String "{data: '赤い箱に近づいて'}"

One step: stop, take a picture taken after stopping, ask, and do the answer
as one short motion of fixed length at speeds the real base can do
(``forward``/``back``/``strafe_*``: ``--speed`` for ``--step`` seconds;
``left``/``right``: ``--turn`` for ``--turn-step`` seconds). The robot never
moves while it waits for an answer. It ends on ``done``, after ``--max-steps``
steps, or on Ctrl-C, and always stops the base.

The model also points at the target in the image; the depth image gives the
distance there, which goes back to the model with the next question. A
``done`` is taken only when that distance is ``--near`` or less: from a camera
0.4 m above the floor, the model said "done" a metre away from the box.

``--dry-run left,forward,done`` answers from that list instead of the VLM, to
try the loop, the motions and the status topics without a key.

The gateway takes one request a second per student and $2 of VLM a day, and
keeps every image for the course staff. A refusal stops the base: a "too
fast" one is waited out a few times, anything else (budget, key) is printed
and ends the program.

Status: ``/vlm_nav/status`` (std_msgs/String), and ``/vlm_nav/overlay``
(rviz_2d_overlay_msgs/OverlayText, for RViz's TextOverlay) when that message
package is installed.
"""

import argparse
import json
import os
import re
import threading
import time

import cv2
import numpy as np
from std_msgs.msg import String

from jedy_skrobot import connect
from jedy_skrobot import VlmRefused
from jedy_skrobot.interface import aita_credentials

#: The AI TA server's models that take images.
MODELS = ('vlm-er', 'vlm-gemini')

#: What the model may answer, and the base twist (vx, vy, vtheta signs) of each.
ACTIONS = {
    'forward': (1, 0, 0),
    'back': (-1, 0, 0),
    'left': (0, 0, 1),
    'right': (0, 0, -1),
    'strafe_left': (0, 1, 0),
    'strafe_right': (0, -1, 0),
    'stop': (0, 0, 0),
    'done': (0, 0, 0),
}
#: The real base's usable speeds (robot-programming-2-2026, 実機に出す前のチェック).
SPEED_RANGE = (0.078, 0.156)   # m/s, forward and sideways
TURN_RANGE = (0.83, 1.66)      # rad/s, turning in place
#: Refusals "too fast" or a failed server waited out before giving up.
RETRIES = 3
#: Answers that are not the JSON asked for, in a row, before giving up.
BAD_ANSWERS = 3
#: Earlier steps told to the model, so it knows what it just did.
HISTORY = 6
#: A forward step is refused when the depth straight ahead is less than this [m].
MIN_AHEAD = 0.25

SYSTEM = """You drive a small wheeled robot (a mecanum base, it can also move sideways) \
by looking through its head camera. Each turn you get the camera image taken just now, \
the goal, and your previous steps. The centre of the image is straight ahead; the \
horizontal field of view is about {fov:.0f} degrees.

Choose exactly ONE action for this step:
- forward: drive {dist:.2f} m forward
- back: drive {dist:.2f} m backward
- left / right: turn in place about {angle:.0f} degrees to the left / right
- strafe_left / strafe_right: slide {dist:.2f} m sideways without turning
- stop: do not move this step (use it when you are unsure)
- done: the goal is achieved; the robot stops

The camera is low, about 0.4 m above the floor, so objects look larger than they \
are: an object 1 m away can already look big. One forward step is only {dist:.2f} m, \
so reaching something 1 m away takes several forward steps.

Each turn also tells you the distance measured now by the depth camera straight \
ahead (the centre of the image); when the target is in the centre, that is the \
distance to it. Give the point of the target in the image as [y, x], normalized \
to 0-1000 (null if it is not visible): the robot measures the distance there too, \
and tells you in the next turn with your previous steps.

Rules: if the target is not in the image, turn to look for it. Turn until the \
target is near the centre of the image before driving towards it. Do not drive \
into walls, furniture or people. For "approach X", say done only when X is about \
{near:.2f} m away or closer (use the distance measured now). If unsure, take one \
more forward step.

Reply with JSON only, no other text and no code fence:
{{"action": "<forward|back|left|right|strafe_left|strafe_right|stop|done>", \
"point": [y, x] or null, "reason": "<one short sentence in Japanese>"}}"""


def parse_answer(text):
    """``(action, reason, point)`` from the model's reply, or None if it is not one.

    ``point`` is the target's ``(y, x)`` in the image, 0-1000, or None.

    Parameters
    ----------
    text : str

    Returns
    -------
    tuple of str or None
    """
    match = re.search(r'\{.*\}', text, re.DOTALL)
    if match is None:
        return None
    try:
        answer = json.loads(match.group(0))
    except ValueError:
        return None
    action = str(answer.get('action', '')).strip().lower()
    if action not in ACTIONS:
        return None
    reason = ' '.join(str(answer.get('reason', '')).split())
    point = answer.get('point')
    try:
        y, x = (float(v) for v in point)
        point = (y, x) if 0 <= y <= 1000 and 0 <= x <= 1000 else None
    except (TypeError, ValueError):
        point = None
    return action, reason, point


def distance_at(depth, point, half=4):
    """Median depth [m] around ``point`` (``(y, x)``, 0-1000), or None.

    None when no pixel there has a reading: on the real D405, anything
    farther than about 0.5 m.
    """
    height, width = depth.shape[:2]
    row = int(round(point[0] / 1000.0 * (height - 1)))
    col = int(round(point[1] / 1000.0 * (width - 1)))
    patch = depth[max(0, row - half):row + half + 1, max(0, col - half):col + half + 1]
    if not np.isfinite(patch).any():
        return None
    return float(np.nanmedian(patch))


def describe(distance):
    """Text for the model about one measured distance."""
    if distance is None:
        return 'distance not measured'
    return 'measured distance to the target {:.2f} m'.format(distance)


def camera_fov_deg(ri):
    """Horizontal field of view of the head camera [deg], from camera_info."""
    K, _ = ri.camera_info()
    rgb, _ = ri.image()
    return float(np.rad2deg(2.0 * np.arctan(rgb.shape[1] / (2.0 * K[0][0]))))


class Status(object):
    """``/vlm_nav/status`` and, if available, the RViz overlay."""

    COLOURS = {'DONE': (0.0, 0.7, 0.0), 'ERROR': (0.8, 0.0, 0.0),
               'THINKING': (0.0, 0.4, 0.8), 'MOVING': (0.0, 0.4, 0.8)}

    def __init__(self, ri):
        self._pub = ri.create_publisher(String, '/vlm_nav/status', 10)
        try:
            from rviz_2d_overlay_msgs.msg import OverlayText
        except ImportError:
            print('(rviz_2d_overlay_msgs がないので RViz の表示 /vlm_nav/overlay は出しません. '
                  '/vlm_nav/status は出ます)')
            OverlayText = None
        self._overlay_type = OverlayText
        self._overlay = (ri.create_publisher(OverlayText, '/vlm_nav/overlay', 10)
                         if OverlayText is not None else None)

    def publish(self, state, goal, text):
        self._pub.publish(String(data='[{}] {}: {}'.format(state, goal, text)))
        if self._overlay is None:
            return
        from std_msgs.msg import ColorRGBA
        msg = self._overlay_type()
        msg.action = self._overlay_type.ADD
        msg.width, msg.height = 600, 130
        msg.horizontal_distance, msg.vertical_distance = 10, 10
        msg.horizontal_alignment = self._overlay_type.LEFT
        msg.vertical_alignment = self._overlay_type.TOP
        r, g, b = self.COLOURS.get(state, (0.4, 0.4, 0.4))
        msg.bg_color = ColorRGBA(r=r, g=g, b=b, a=0.8)
        msg.fg_color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        msg.line_width = 2
        msg.text_size = 14.0
        msg.font = 'DejaVu Sans Mono'
        msg.text = 'VLM nav [{}]\ngoal: {}\n{}'.format(state, goal, text)
        self._overlay.publish(msg)


class Goals(object):
    """The goal from the command line and from ``/vlm_nav/prompt``."""

    def __init__(self, ri, first=None):
        self._lock = threading.Lock()
        self._new = threading.Event()
        self._goal = first
        if first:
            self._new.set()
        ri.create_subscription(String, '/vlm_nav/prompt', self._on_prompt, 10)

    def _on_prompt(self, msg):
        text = msg.data.strip()
        if not text:
            return
        with self._lock:
            self._goal = text
        self._new.set()

    def changed(self):
        """Whether a goal arrived that has not been taken yet."""
        return self._new.is_set()

    def take(self, timeout=None):
        """The newest goal, waiting for one; None on timeout."""
        if not self._new.wait(timeout):
            return None
        with self._lock:
            self._new.clear()
            return self._goal


class ScriptedAnswers(object):
    """``--dry-run``: answers from a fixed list instead of the VLM.

    For trying the control loop, the status topic and the motions without
    a key or spending anything. Nothing is sent anywhere; every answer says
    it is scripted, so a dry run is never mistaken for the model's choice.

    Parameters
    ----------
    actions : list of str
        Actions in :data:`ACTIONS`, answered in order; after the last one,
        ``done``.
    """

    def __init__(self, actions):
        self._actions = list(actions)
        self.model = 'dry-run'
        self.last_cost = self.total_cost = self.last_latency = self.last_wait = 0.0

    def ask(self, text, image, system):
        action = self._actions.pop(0) if self._actions else 'done'
        return json.dumps({'action': action,
                           'reason': '[dry-run] 台本どおり (VLM には聞いていない)'},
                          ensure_ascii=False)


class RobotAnswers(object):
    """The VLM's answers, through ``ri.ask`` (the AI TA server)."""

    def __init__(self, ri, model, reasoning):
        self._ri = ri
        self.model = model
        self.reasoning = reasoning

    def ask(self, text, image, system):
        return self._ri.ask(text, image, model=self.model, system=system,
                            reasoning=self.reasoning)

    @property
    def last_cost(self):
        return self._ri.vlm_last_cost

    @property
    def total_cost(self):
        return self._ri.vlm_total_cost

    @property
    def last_latency(self):
        return self._ri.vlm_last_latency

    @property
    def last_wait(self):
        return self._ri.vlm_last_wait


def print_timing(timing):
    """One line: how a step's time (wall clock) splits up."""
    if not timing:
        return
    t = np.array(timing)
    period = t[:, 0] + t[:, 1] + t[:, 3]
    print('時間 (壁時計, {} 手): 1 手 中央値 {:.1f} s / 最大 {:.1f} s = 止めて撮る {:.1f} s '
          '+ VLM {:.1f} s (最大 {:.1f} s) + 動く {:.1f} s．1 秒制限で待った合計 {:.1f} s'.format(
              len(t), np.median(period), period.max(), np.median(t[:, 0]),
              np.median(t[:, 1]), t[:, 1].max(), np.median(t[:, 3]), t[:, 2].sum()))


def check_range(name, value, low, high):
    if not low <= value <= high:
        raise SystemExit(
            '{} {} は実機の台車が動ける範囲 {}〜{} の外です '
            '(遅すぎると車輪が回らず，速すぎると上限で止められる)'.format(
                name, value, low, high))


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('goal', nargs='*',
                   help='what to do, in words (e.g. 赤い箱に近づいて). '
                        'Without it: wait for /vlm_nav/prompt')
    p.add_argument('--model', default='vlm-er', choices=MODELS,
                   help='vlm-er = Gemini Robotics-ER, vlm-gemini = Gemini')
    # 'low', not ri.ask's 'none': with 'none' vlm-er turned left and right
    # around the box for 7 extra steps (Gazebo, 2026-11-14); with 'low' it
    # reached it in 7 steps, about 1 s more per step.
    p.add_argument('--reasoning', default='low', choices=('none', 'low', 'medium', 'high'),
                   help="how much the model thinks before answering ('none' is fastest)")
    p.add_argument('--max-steps', type=int, default=30,
                   help='give up after this many steps (requests) per goal')
    p.add_argument('--speed', type=float, default=0.12,
                   help='forward/back/sideways speed [m/s], 0.078-0.156')
    p.add_argument('--step', type=float, default=1.5,
                   help='seconds one forward/back/sideways step drives')
    p.add_argument('--turn', type=float, default=1.0,
                   help='turning speed [rad/s], 0.83-1.66')
    p.add_argument('--turn-step', type=float, default=0.4,
                   help='seconds one left/right step turns')
    p.add_argument('--near', type=float, default=0.45,
                   help='"done" only when the depth camera measures the target '
                        'this close [m] (when there is a depth image)')
    p.add_argument('--listen', action='store_true',
                   help='with a goal on the command line: after it, keep '
                        'waiting for /vlm_nav/prompt instead of exiting')
    p.add_argument('--say', action='store_true',
                   help='say each reason with ri.say (VOICEVOX)')
    p.add_argument('--save', metavar='DIR', default=None,
                   help='write each image sent to the VLM to DIR/step_NN.jpg')
    p.add_argument('--dry-run', metavar='ACTIONS', default=None,
                   help='do not ask the VLM: answer these comma-separated '
                        'actions in order, then done (e.g. left,forward,forward), '
                        'to try the loop without a key')
    args = p.parse_args()
    check_range('--speed', args.speed, *SPEED_RANGE)
    check_range('--turn', args.turn, *TURN_RANGE)
    if not 0.0 < args.step <= 3.0 or not 0.0 < args.turn_step <= 1.5:
        raise SystemExit('--step は 0〜3 秒，--turn-step は 0〜1.5 秒にしてください')

    if args.dry_run is not None:
        script = [a.strip() for a in args.dry_run.split(',') if a.strip()]
        unknown = [a for a in script if a not in ACTIONS]
        if unknown:
            raise SystemExit('--dry-run に知らない動きがあります: {} (使えるのは {})'.format(
                ', '.join(unknown), ', '.join(ACTIONS)))
        vlm = ScriptedAnswers(script)
        print('[dry-run] VLM には聞かず，台本 {} のとおりに動きます'.format(script))
    else:
        try:
            aita_credentials()   # the key, before anything moves
        except VlmRefused as exc:
            raise SystemExit(str(exc))
    first = ' '.join(args.goal) or None

    robot, ri = connect()
    if args.dry_run is None:
        vlm = RobotAnswers(ri, args.model, args.reasoning)
    if not ri.has_camera():
        raise SystemExit('頭のカメラの画像 (/camera/color/image_rect_raw) が来ていません. '
                         'Gazebo は render_sensors:=true，実機は camera:=true で起動する')
    ri.servo_on()
    ri.tuck_arms()   # arms up before driving: hanging, they drag on the floor
    status = Status(ri)
    goals = Goals(ri, first)
    system = SYSTEM.format(fov=camera_fov_deg(ri), dist=args.speed * args.step,
                           angle=np.rad2deg(args.turn * args.turn_step), near=args.near)
    has_depth = ri.count_publishers('/camera/depth/image_rect_raw') > 0
    if not has_depth:
        print('距離画像が無いので，done を距離で確かめません')
    say = args.say

    try:
        while True:
            if not goals.changed():
                print('目標を待っています: ros2 topic pub --once /vlm_nav/prompt '
                      'std_msgs/msg/String "{data: \'赤い箱に近づいて\'}"')
            goal = goals.take()
            print('目標: {}  (モデル {}, 最大 {} 回)'.format(goal, vlm.model, args.max_steps))
            history = []
            bad = retries = 0
            step = 0
            result = 'max_steps'
            timing = []   # per step: stop + capture, model call, of it waiting, motion [s]
            while step < args.max_steps:
                if goals.changed():
                    result = 'new goal'
                    break
                t0 = time.monotonic()
                ri.stop()
                stopped = ri.now()
                rgb, _ = ri.image(newer_than=stopped)   # a picture after stopping
                depth = ri.depth_image(newer_than=stopped)[0] if has_depth else None
                t1 = time.monotonic()
                status.publish('THINKING', goal, 'step {}: asking {}'.format(
                    step + 1, vlm.model))
                ahead = None
                if depth is not None:
                    ahead = distance_at(depth, (500.0, 500.0), half=10)
                text = ('Goal: {}\nDistance straight ahead now: {}\n'
                        'Previous steps (oldest first): {}'.format(
                            goal, 'not measured' if ahead is None else '{:.2f} m'.format(ahead),
                            '; '.join(history[-HISTORY:]) or 'none'))
                try:
                    reply = vlm.ask(text, rgb, system)
                    t2 = time.monotonic()
                except VlmRefused as exc:
                    ri.stop()
                    status.publish('ERROR', goal, str(exc))
                    print('\n' + str(exc))
                    if exc.retryable and retries < RETRIES:
                        retries += 1
                        wait = 2.0 ** retries
                        print('台車を止めて {:.0f} 秒待ってからやり直します ({}/{})'.format(
                            wait, retries, RETRIES))
                        ri.sleep(wait)
                        continue
                    raise SystemExit('VLM を使えないので終わります (台車は止めました)')
                retries = 0
                step += 1
                if args.save:
                    os.makedirs(args.save, exist_ok=True)
                    cv2.imwrite(os.path.join(args.save, 'step_{:02d}.jpg'.format(step)),
                                cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
                answer = parse_answer(reply)
                if answer is None:
                    bad += 1
                    print('[{}/{}] 答えが JSON ではありません (止まったまま): {!r}'.format(
                        step, args.max_steps, reply[:200]))
                    if bad >= BAD_ANSWERS:
                        result = 'bad answers'
                        break
                    continue
                bad = 0
                action, reason, point = answer
                distance = None
                if depth is not None and point is not None:
                    distance = distance_at(depth, point)
                print('[{}/{}] {:<12s} {}  ({}{:.1f} s, ${:.4f}, 合計 ${:.4f})'.format(
                    step, args.max_steps, action, reason,
                    '' if distance is None else '距離 {:.2f} m, '.format(distance),
                    vlm.last_latency, vlm.last_cost, vlm.total_cost), flush=True)
                if action == 'done' and depth is not None and args.dry_run is None and (
                        distance is None or distance > args.near):
                    # Checked, not trusted: a low camera makes things look near.
                    print('    -> まだ遠い ({}): done にせず続けます'.format(
                        '距離が測れない' if distance is None
                        else '{:.2f} m > {:.2f} m'.format(distance, args.near)))
                    history.append('done refused by the robot: {} (needs {:.2f} m or '
                                   'less)'.format(describe(distance), args.near))
                    status.publish('MOVING', goal, 'done? still {}'.format(describe(distance)))
                    continue
                history.append('{} ({}; {})'.format(action, reason, describe(distance)))
                status.publish('DONE' if action == 'done' else 'MOVING', goal,
                               '{}: {}'.format(action, reason))
                if say:
                    try:
                        ri.say(reason, wait=False)
                    except RuntimeError as exc:
                        print('ri.say が使えません ({}). 以後しゃべりません'.format(exc))
                        say = False
                if action == 'done':
                    timing.append((t1 - t0, t2 - t1, vlm.last_wait, 0.0))
                    result = 'done'
                    break
                if action == 'forward' and ahead is not None and ahead < MIN_AHEAD:
                    # Checked, not trusted: one step must not drive into what is ahead.
                    print('    -> 前 {:.2f} m に物があるので前進しません'.format(ahead))
                    history.append('forward refused by the robot: something {:.2f} m '
                                   'straight ahead'.format(ahead))
                    continue
                sx, sy, st = ACTIONS[action]
                duration = args.turn_step if st else args.step
                if sx or sy or st:
                    ri.go_velocity(sx * args.speed, sy * args.speed, st * args.turn,
                                   duration=duration)
                timing.append((t1 - t0, t2 - t1, vlm.last_wait, time.monotonic() - t2))
            ri.stop()
            print('終わり: {} ({} 回, 合計 ${:.4f})'.format(result, step, vlm.total_cost))
            print_timing(timing)
            if result != 'new goal':
                status.publish('DONE' if result == 'done' else 'STOPPED', goal, result)
            if first is not None and not args.listen and result != 'new goal':
                break
    except KeyboardInterrupt:
        print('\nCtrl-C: 止めます')
    finally:
        ri.stop()   # whatever happened, the base gets a stop


if __name__ == '__main__':
    main()
