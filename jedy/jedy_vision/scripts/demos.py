#!/usr/bin/env python3
"""List the Jedy demos: what each shows, and how to start it.

    ros2 run jedy_vision demos
"""

GAZEBO = 'ros2 launch jedy_bringup jedy_gazebo.launch.py'
STAGE = GAZEBO + ' demo:=true person:=true camera:=d405'
REAL = 'ros2 launch jedy_bringup jedy_fullbody_all.launch.py camera:=true'

#: Groups of (command after ``ros2 run jedy_vision``, what it does, what it
#: teaches); all run on STAGE.

DEMOS = [
    ('AprilTag', [
        ('tag_pose', 'タグの位置と向きをロボット座標で表示 (RViz: /demo/markers)',
         'カメラ内部パラメータ, PnP, 座標変換'),
        ('approach_tag', 'タグの正面 0.2 m に向きを合わせて止まる',
         '極座標の姿勢制御, 閉ループ'),
        ('touch_tag', '手でタグにタッチ (手前で一度止まってから)',
         'カメラ座標→ロボット座標, 逆運動学, 経由点'),
        ('grasp_tag', '全面タグの立方体を片手でつかんで持ち上げ, 戻す',
         '複数タグから物体の姿勢, 姿勢付き逆運動学, 把持の確認'),
    ]),
    ('もの', [
        ('find_object cup', '見回してコップを探し, 近づいて指さす (chair, person も可)',
         '探索, 床との交点で距離推定, D405 の深度の限界'),
    ]),
    ('人', [
        ('detect_people --pose', 'いまの画像の人と骨格を画像に保存', 'YOLO, MediaPipe'),
        ('look_at_person', '首で人を追う', 'コントローラ単位の指令'),
        ('follow_person', '人の方へ回って近づき 1 m で止まる', '視覚フィードバック'),
        ('mimic_arms', '人の腕を鏡のようにまねる', '姿勢推定→逆運動学'),
        ('wave_back', '手を挙げた人に手を振り返して「こんにちは」', '姿勢の判定, 動作の生成'),
    ]),
    ('声 (VOICEVOX と音声認識)', [
        ('voice_demo', '「コップを探して」「ついてきて」「止まって」で上のデモを切り替え',
         '音声認識→行動選択'),
    ]),
]

#: The same for ``ros2 run jedy_skrobot``, each with the Gazebo world it
#: needs and a note (or None).
SKROBOT_DEMOS = [
    ('jedy_skrobot (scripts/enshu)', [
        ('vlm_navigate 赤い箱に近づいて', '言葉で言った目標へ, VLM に聞きながら走る',
         'VLM で次の一手を決める',
         GAZEBO + ' box:=true',
         'VLM のキーが要る: ta login か, 自分のキーで export GOOGLE_API_KEY=...'),
        ('color_clusters', '赤い物の 3 次元の位置 (--hue で色を変える)',
         '色で画素を選び, 深度画像で 3 次元に戻す',
         GAZEBO + ' box:=true',
         'camera:=d405 は付けない (箱が D405 の深度 0.5 m より遠い)'),
        ('checkerboard_pose', 'チェッカーボードの位置と向き',
         '既知の模様から姿勢 (PnP)',
         GAZEBO + ' checkerboard:=true', None),
        ('checkpoint_go_forward_imu', '走って回り, 傾いたら止まる',
         'IMU (Gazebo のみ. 実機の RCB-4mini には無い)',
         GAZEBO, None),
    ]),
]


def main():
    print('Gazebo の舞台:  ' + STAGE + '  (jedy_vision のデモ)')
    print('実機 (D405):    ' + REAL)
    print('声を使うなら:   ros2 launch voicevox voicevox_texttospeech.launch.xml')
    print('                ros2 launch ros_speech_recognition speech_recognition.launch.xml language:=ja-JP\n')
    for group, items in DEMOS:
        print('[{}]'.format(group))
        for cmd, what, learn in items:
            print('  ros2 run jedy_vision {:22s} {}'.format(cmd, what))
            print('  {:42s} 学べること: {}'.format('', learn))
        print()
    for group, items in SKROBOT_DEMOS:
        print('[{}]'.format(group))
        for cmd, what, learn, world, note in items:
            print('  ros2 run jedy_skrobot ' + cmd)
            print('  {:42s} {}'.format('', what))
            print('  {:42s} 学べること: {}'.format('', learn))
            print('  {:42s} Gazebo: {}'.format('', world))
            if note:
                print('  {:42s} {}'.format('', note))
        print()
    print('実機の台車は 0.078 m/s 未満では動かない / D405 の深度は 0.07-0.5 m だけ (デモはそれを前提)')


if __name__ == '__main__':
    main()
