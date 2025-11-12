#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
from scipy.spatial.transform import Rotation as R


class CheckerboardDetector(Node):
    def __init__(self):
        super().__init__('checkerboard_detector')

        # Parameters
        self.declare_parameter('input_topic', '/remote/color/image_rect_raw')
        self.declare_parameter('camera_info_topic', '/remote/color/camera_info')
        self.declare_parameter('checkerboard_size', [7, 4])  # 内部コーナー数
        self.declare_parameter('square_size', 0.02)  # チェッカーボードの1マスのサイズ [m]
        self.declare_parameter('checkerboard_frame', 'checkerboard')

        input_topic = self.get_parameter('input_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.checkerboard_size = tuple(self.get_parameter('checkerboard_size').value)
        self.square_size = self.get_parameter('square_size').value
        self.checkerboard_frame = self.get_parameter('checkerboard_frame').value

        # CV Bridge
        self.bridge = CvBridge()

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # カメラ内部パラメータ（camera_infoから取得）
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_received = False

        # 3Dオブジェクトポイントの準備
        self.object_points = np.zeros((self.checkerboard_size[0] * self.checkerboard_size[1], 3), np.float32)
        self.object_points[:, :2] = np.mgrid[0:self.checkerboard_size[0],
                                               0:self.checkerboard_size[1]].T.reshape(-1, 2)
        self.object_points *= self.square_size

        # Subscriber for camera info
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            10)

        # Subscriber for image
        self.subscription = self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            10)

        # Publisher for visualization image
        self.image_pub = self.create_publisher(Image, '~/debug_image', 10)

        self.get_logger().info(f'Checkerboard detector started: {input_topic}')
        self.get_logger().info(f'Camera info topic: {camera_info_topic}')
        self.get_logger().info(f'Checkerboard size: {self.checkerboard_size}, Square size: {self.square_size}m')

    def camera_info_callback(self, msg):
        """camera_infoからカメラパラメータを取得"""
        if not self.camera_info_received:
            # カメラ行列を取得
            self.camera_matrix = np.array(msg.k).reshape(3, 3)

            # 歪み係数を取得
            self.dist_coeffs = np.array(msg.d)

            self.camera_info_received = True
            self.get_logger().info('Camera info received and parameters updated')
            self.get_logger().info(f'Camera matrix:\n{self.camera_matrix}')
            self.get_logger().info(f'Distortion coefficients: {self.dist_coeffs}')

    def image_callback(self, msg):
        # camera_infoが受信されるまで処理をスキップ
        if not self.camera_info_received:
            self.get_logger().warn('Waiting for camera_info...', throttle_duration_sec=5.0)
            return

        try:
            # ROS Image -> OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # 可視化用の画像をコピー
            debug_image = cv_image.copy()

            # チェッカーボード検出
            ret, corners = cv2.findChessboardCorners(gray, self.checkerboard_size, None)

            if ret:
                # サブピクセル精度でコーナーを改良
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

                # チェッカーボードのコーナーを描画
                cv2.drawChessboardCorners(debug_image, self.checkerboard_size, corners_refined, ret)

                # 姿勢推定
                success, rotation_vector, translation_vector = cv2.solvePnP(
                    self.object_points,
                    corners_refined,
                    self.camera_matrix,
                    self.dist_coeffs
                )

                if success:
                    # 座標軸を描画
                    axis_length = self.square_size * 3
                    axis = np.float32([
                        [0, 0, 0],
                        [axis_length, 0, 0],
                        [0, axis_length, 0],
                        [0, 0, axis_length]
                    ]).reshape(-1, 3)

                    img_points, _ = cv2.projectPoints(
                        axis,
                        rotation_vector,
                        translation_vector,
                        self.camera_matrix,
                        self.dist_coeffs
                    )

                    img_points = img_points.astype(int)
                    origin = tuple(img_points[0].ravel())

                    # X軸 (赤)
                    cv2.line(debug_image, origin, tuple(img_points[1].ravel()), (0, 0, 255), 3)
                    # Y軸 (緑)
                    cv2.line(debug_image, origin, tuple(img_points[2].ravel()), (0, 255, 0), 3)
                    # Z軸 (青)
                    cv2.line(debug_image, origin, tuple(img_points[3].ravel()), (255, 0, 0), 3)

                    # 位置情報をテキストで表示
                    text = f'X: {translation_vector[0][0]:.3f}m Y: {translation_vector[1][0]:.3f}m Z: {translation_vector[2][0]:.3f}m'
                    cv2.putText(debug_image, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                    # TF配信 (画像のframe_idを使用)
                    self.publish_tf(msg.header.stamp, msg.header.frame_id, rotation_vector, translation_vector)
                    self.get_logger().info('Checkerboard detected and TF published', throttle_duration_sec=1.0)
                else:
                    self.get_logger().warn('PnP solve failed', throttle_duration_sec=1.0)
                    cv2.putText(debug_image, 'PnP solve failed', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                self.get_logger().debug('Checkerboard not found', throttle_duration_sec=1.0)
                cv2.putText(debug_image, 'Checkerboard not found', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # デバッグ画像をpublish
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            debug_msg.header = msg.header
            self.image_pub.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f'Error in image callback: {str(e)}')

    def publish_tf(self, timestamp, camera_frame, rotation_vector, translation_vector):
        # TransformStampedメッセージの作成
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = camera_frame
        t.child_frame_id = self.checkerboard_frame

        # 並進
        t.transform.translation.x = float(translation_vector[0][0])
        t.transform.translation.y = float(translation_vector[1][0])
        t.transform.translation.z = float(translation_vector[2][0])

        # 回転（ロドリゲス -> 回転行列 -> クォータニオン）
        rotation_matrix, _ = cv2.Rodrigues(rotation_vector)

        # 回転行列からクォータニオンへ変換
        r = R.from_matrix(rotation_matrix)
        quat = r.as_quat()  # [x, y, z, w]

        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])

        # デバッグログ
        self.get_logger().debug(
            f'Publishing TF: {camera_frame} -> {self.checkerboard_frame}, '
            f'pos: [{t.transform.translation.x:.3f}, {t.transform.translation.y:.3f}, {t.transform.translation.z:.3f}], '
            f'rot: [{t.transform.rotation.x:.3f}, {t.transform.rotation.y:.3f}, {t.transform.rotation.z:.3f}, {t.transform.rotation.w:.3f}]'
        )

        # TF配信
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = CheckerboardDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
