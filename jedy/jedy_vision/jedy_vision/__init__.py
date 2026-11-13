"""People in Jedy's head camera: YOLO person boxes and MediaPipe body pose.

Both take the RGB image that ``ri.image()`` returns::

    from jedy_skrobot import connect
    from jedy_vision import PersonDetector, PoseEstimator

    detector = PersonDetector()                  # before connect(): see below
    robot, ri = connect()
    rgb, stamp = ri.image()
    people = detector.detect(rgb)                # [Person(box, score), ...]
    with PoseEstimator() as estimator:
        poses = estimator.estimate(rgb)          # [Pose, ...] (33 landmarks)

The model files are downloaded at build time into
``share/jedy_vision/models`` (see CMakeLists.txt); nothing is fetched at run
time.

Make the YOLO detectors before ``connect()``. They load the model with a
first, slow detection, and once connected that runs ~8x slower: ``connect()``
spins a thread that takes ~900 messages/s in Python (Gazebo: ``/clock``
200 Hz, ``joint_states``, ``imu`` and five ``controller_state`` at 100 Hz),
and every switch back to it waits for the GIL. Measured in Gazebo on a 48-core
PC: first detect 2.0 s before, 14.6-17 s after; later detections 0.04 s
before, 0.09-0.2 s after.
"""

import os
import warnings
from dataclasses import dataclass

import numpy as np

# Ultralytics phones home (version check, fonts) unless told it is offline.
os.environ.setdefault("YOLO_OFFLINE", "1")
os.environ.setdefault("YOLO_VERBOSE", "False")
# ultralytics imports the venv's matplotlib next to the system one, whose
# mpl_toolkits it finds first; 3-D plotting is not used here.
warnings.filterwarnings("ignore", message="Unable to import Axes3D")

#: COCO class id of "person" in YOLO's detection head.
PERSON_CLASS = 0
#: MediaPipe Pose landmark indices used by the demos (33 in total).
NOSE = 0
LEFT_SHOULDER, RIGHT_SHOULDER = 11, 12
LEFT_ELBOW, RIGHT_ELBOW = 13, 14
LEFT_WRIST, RIGHT_WRIST = 15, 16
LEFT_HIP, RIGHT_HIP = 23, 24


def models_dir():
    """Directory holding the downloaded model files.

    Returns
    -------
    str
        ``<prefix>/share/jedy_vision/models``.
    """
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory("jedy_vision"), "models")


def model_path(name):
    """Path of one model file, failing loudly if the build did not fetch it.

    Parameters
    ----------
    name : str
        File name, e.g. ``yolo11n.pt``.

    Returns
    -------
    str
    """
    path = os.path.join(models_dir(), name)
    if not os.path.isfile(path):
        raise FileNotFoundError(
            "{} not found. It is downloaded by `colcon build --packages-select "
            "jedy_vision`; rebuild that package (needs network once).".format(path))
    return path


@dataclass
class Person:
    """One detected person.

    Attributes
    ----------
    box : tuple of float
        ``(x1, y1, x2, y2)`` [px].
    score : float
        Detection confidence, 0..1.
    """

    box: tuple
    score: float

    @property
    def center(self):
        """Centre of the box ``(x, y)`` [px]."""
        x1, y1, x2, y2 = self.box
        return (0.5 * (x1 + x2), 0.5 * (y1 + y2))

    @property
    def area(self):
        """Box area [px^2]."""
        x1, y1, x2, y2 = self.box
        return max(0.0, x2 - x1) * max(0.0, y2 - y1)


class PersonDetector:
    """YOLO (``yolo11n.pt``) restricted to the person class.

    Parameters
    ----------
    weights : str
        Model file name in :func:`models_dir`.
    min_score : float
        Detections below this confidence are dropped.
    """

    def __init__(self, weights="yolo11n.pt", min_score=0.4):
        from ultralytics import YOLO
        self.model = YOLO(model_path(weights))
        self.min_score = min_score
        self.warm_up()

    def warm_up(self, shape=(480, 640, 3)):
        """Run once on a blank image: the first call loads the model (seconds).

        ``__init__`` already does; make the detector before ``connect()``
        (see the module docstring).
        """
        self.detect(np.zeros(shape, dtype=np.uint8))

    def detect(self, rgb):
        """People in an RGB image, largest first.

        Parameters
        ----------
        rgb : numpy.ndarray
            ``(H, W, 3)`` uint8 RGB, as ``ri.image()`` returns.

        Returns
        -------
        list of Person
        """
        # Ultralytics treats numpy input as OpenCV's BGR.
        result = self.model(np.ascontiguousarray(rgb[:, :, ::-1]),
                            classes=[PERSON_CLASS], conf=self.min_score,
                            verbose=False)[0]
        people = [Person(tuple(float(v) for v in xyxy), float(score))
                  for xyxy, score in zip(result.boxes.xyxy.cpu().numpy(),
                                         result.boxes.conf.cpu().numpy())]
        return sorted(people, key=lambda p: p.area, reverse=True)


@dataclass
class Pose:
    """One person's body landmarks from MediaPipe Pose.

    Attributes
    ----------
    xy : numpy.ndarray
        ``(33, 2)`` landmark positions in the image [px].
    visibility : numpy.ndarray
        ``(33,)`` how likely each landmark is visible, 0..1.
    """

    xy: np.ndarray
    visibility: np.ndarray

    def point(self, index, min_visibility=0.5):
        """One landmark ``(x, y)`` [px], or None if it is not visible enough."""
        if self.visibility[index] < min_visibility:
            return None
        return self.xy[index]


class PoseEstimator:
    """MediaPipe Pose Landmarker (lite) on single images.

    Parameters
    ----------
    max_people : int
        How many people to look for.
    """

    def __init__(self, max_people=1):
        import mediapipe as mp
        from mediapipe.tasks.python import BaseOptions, vision
        self._mp = mp
        options = vision.PoseLandmarkerOptions(
            base_options=BaseOptions(
                model_asset_path=model_path("pose_landmarker_lite.task")),
            running_mode=vision.RunningMode.IMAGE,
            num_poses=max_people)
        self._landmarker = vision.PoseLandmarker.create_from_options(options)

    # Use it as ``with PoseEstimator() as estimator:``. MediaPipe's own
    # __del__ at interpreter exit runs after its thread pool is gone and
    # raises ("cannot schedule new futures after shutdown"), and so would an
    # atexit hook, so it has to be closed before the program ends.
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    def estimate(self, rgb):
        """Body landmarks of the people in an RGB image.

        Parameters
        ----------
        rgb : numpy.ndarray
            ``(H, W, 3)`` uint8 RGB.

        Returns
        -------
        list of Pose
        """
        height, width = rgb.shape[:2]
        image = self._mp.Image(image_format=self._mp.ImageFormat.SRGB,
                               data=np.ascontiguousarray(rgb))
        result = self._landmarker.detect(image)
        poses = []
        for landmarks in result.pose_landmarks:
            xy = np.array([[lm.x * width, lm.y * height] for lm in landmarks])
            vis = np.array([lm.visibility for lm in landmarks])
            poses.append(Pose(xy, vis))
        return poses

    def close(self):
        """Release the landmarker. Call before the program exits."""
        if self._landmarker is not None:
            self._landmarker.close()
            self._landmarker = None


#: MediaPipe Pose landmarks on the face: nose, eyes, ears, mouth.
POSE_FACE = tuple(range(0, 11))
#: Side of the square the head is upscaled to for the face models [px].
FACE_CROP = 256


@dataclass
class Face:
    """One face, found in a head crop and mapped back to the image.

    Attributes
    ----------
    box : tuple of float
        ``(x1, y1, x2, y2)`` [px] in the image.
    score : float
        The face detector's confidence, 0..1.
    landmarks : numpy.ndarray or None
        ``(478, 2)`` face mesh points [px] in the image, or None if the
        face landmarker found none in the crop.
    """

    box: tuple
    score: float
    landmarks: object


class FaceEstimator:
    """Faces of the people a :class:`Pose` was found for.

    A face 1.5-2.5 m from the head camera is about 30 px wide: too small for
    MediaPipe's face models on the whole image (on Gazebo's person they found
    none, or the hand). So each face is looked for in a crop around the head
    (the Pose's face landmarks), upscaled to :data:`FACE_CROP` px: there
    the face detector (BlazeFace) and the face landmarker (478 points) work.
    Use as ``with FaceEstimator() as faces:``, like :class:`PoseEstimator`.
    """

    def __init__(self, min_score=0.5):
        import mediapipe as mp
        from mediapipe.tasks.python import BaseOptions, vision
        self._mp = mp
        self._detector = vision.FaceDetector.create_from_options(
            vision.FaceDetectorOptions(
                base_options=BaseOptions(
                    model_asset_path=model_path("blaze_face_short_range.tflite")),
                min_detection_confidence=min_score))
        self._landmarker = vision.FaceLandmarker.create_from_options(
            vision.FaceLandmarkerOptions(
                base_options=BaseOptions(
                    model_asset_path=model_path("face_landmarker.task")),
                num_faces=1))

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    def estimate(self, rgb, poses):
        """The face of each pose, where one is found.

        Parameters
        ----------
        rgb : numpy.ndarray
            ``(H, W, 3)`` uint8 RGB.
        poses : list of Pose
            From :meth:`PoseEstimator.estimate` on the same image.

        Returns
        -------
        list of Face
            At most one per pose; poses whose face is not found are left out.
        """
        import cv2
        h, w = rgb.shape[:2]
        faces = []
        for pose in poses:
            points = [pose.point(i) for i in POSE_FACE]
            points = np.array([p for p in points if p is not None])
            if len(points) < 3:
                continue
            centre = points.mean(axis=0)
            half = max(2.0 * np.ptp(points[:, 0]), 12.0)
            x0, y0 = int(max(centre[0] - half, 0)), int(max(centre[1] - half, 0))
            x1, y1 = int(min(centre[0] + half, w)), int(min(centre[1] + half, h))
            if x1 - x0 < 8 or y1 - y0 < 8:
                continue
            crop = cv2.resize(rgb[y0:y1, x0:x1], (FACE_CROP, FACE_CROP),
                              interpolation=cv2.INTER_CUBIC)
            sx, sy = (x1 - x0) / float(FACE_CROP), (y1 - y0) / float(FACE_CROP)
            image = self._mp.Image(image_format=self._mp.ImageFormat.SRGB,
                                   data=np.ascontiguousarray(crop))
            found = self._detector.detect(image).detections
            if not found:
                continue
            best = max(found, key=lambda d: d.categories[0].score)
            b = best.bounding_box
            box = (x0 + b.origin_x * sx, y0 + b.origin_y * sy,
                   x0 + (b.origin_x + b.width) * sx, y0 + (b.origin_y + b.height) * sy)
            mesh = self._landmarker.detect(image).face_landmarks
            landmarks = None
            if mesh:
                landmarks = np.array([[x0 + lm.x * FACE_CROP * sx, y0 + lm.y * FACE_CROP * sy]
                                      for lm in mesh[0]])
            faces.append(Face(box, float(best.categories[0].score), landmarks))
        return faces

    def close(self):
        """Release the models. Call before the program exits."""
        for name in ('_detector', '_landmarker'):
            model = getattr(self, name)
            if model is not None:
                model.close()
                setattr(self, name, None)


#: Bones of a hand drawn by detect_people (MediaPipe Hands indices: 0 wrist,
#: then 4 per finger from the base, thumb first).
HAND_BONES = [(0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8),
              (5, 9), (9, 10), (10, 11), (11, 12), (9, 13), (13, 14), (14, 15),
              (15, 16), (13, 17), (0, 17), (17, 18), (18, 19), (19, 20)]


@dataclass
class Hand:
    """One hand.

    Attributes
    ----------
    landmarks : numpy.ndarray
        ``(21, 2)`` [px]: the wrist, then four points per finger, thumb first.
    side : str
        ``'Left'`` or ``'Right'``, as MediaPipe reports it (the person's own
        side for a camera facing them).
    score : float
        Confidence of ``side``, 0..1.
    """

    landmarks: np.ndarray
    side: str
    score: float


class HandEstimator:
    """MediaPipe Hand Landmarker (21 points a hand) on single images.

    Hands are found in the whole image (on Gazebo's person 1.2-2.4 m away it
    found one or both). Use as ``with HandEstimator() as hands:``.
    """

    def __init__(self, max_hands=2, min_score=0.3):
        import mediapipe as mp
        from mediapipe.tasks.python import BaseOptions, vision
        self._mp = mp
        self._landmarker = vision.HandLandmarker.create_from_options(
            vision.HandLandmarkerOptions(
                base_options=BaseOptions(model_asset_path=model_path("hand_landmarker.task")),
                num_hands=max_hands, min_hand_detection_confidence=min_score))

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    def estimate(self, rgb):
        """The hands in an RGB image.

        Returns
        -------
        list of Hand
        """
        h, w = rgb.shape[:2]
        image = self._mp.Image(image_format=self._mp.ImageFormat.SRGB,
                               data=np.ascontiguousarray(rgb))
        result = self._landmarker.detect(image)
        hands = []
        for landmarks, handedness in zip(result.hand_landmarks, result.handedness):
            xy = np.array([[lm.x * w, lm.y * h] for lm in landmarks])
            hands.append(Hand(xy, handedness[0].category_name, float(handedness[0].score)))
        return hands

    def close(self):
        """Release the landmarker. Call before the program exits."""
        if self._landmarker is not None:
            self._landmarker.close()
            self._landmarker = None


def box_distance(depth, box, shrink=0.3):
    """Median depth inside the middle of a box [m], or None if no reading.

    Parameters
    ----------
    depth : numpy.ndarray
        ``(H, W)`` float32 metres, NaN where there is no reading
        (``ri.depth_image()``).
    box : tuple of float
        ``(x1, y1, x2, y2)`` [px].
    shrink : float
        Use only the central part of the box (``1 - 2*shrink`` of each side),
        where the person is, not the background around them.

    Returns
    -------
    float or None
    """
    x1, y1, x2, y2 = box
    w, h = x2 - x1, y2 - y1
    xa, xb = int(x1 + shrink * w), int(x2 - shrink * w)
    ya, yb = int(y1 + shrink * h), int(y2 - shrink * h)
    patch = depth[max(ya, 0):max(yb, ya + 1), max(xa, 0):max(xb, xa + 1)]
    finite = patch[np.isfinite(patch)]
    if finite.size == 0:
        return None
    return float(np.median(finite))


# ------------------------------------------------------------------ objects

class ObjectDetector:
    """YOLO (``yolo11s.pt``) for any of its 80 COCO classes.

    The small model, not the nano one: on Gazebo's renders yolo11n gave a
    cup 0.21 and yolo11s 0.78, at 145 ms instead of 60 ms on a laptop CPU.

    Parameters
    ----------
    min_score : float
        Detections below this confidence are dropped.
    """

    def __init__(self, min_score=0.3, weights="yolo11s.pt"):
        from ultralytics import YOLO
        self.model = YOLO(model_path(weights))
        self.min_score = min_score
        self.names = self.model.names  # {id: "cup", ...}
        # Load the model now (see PersonDetector.warm_up).
        self.detect(np.zeros((480, 640, 3), dtype=np.uint8))

    def class_id(self, name):
        """COCO class id of an English name like ``cup``; raises if unknown."""
        for i, n in self.names.items():
            if n == name:
                return i
        raise ValueError("unknown class {!r}; YOLO knows: {}".format(
            name, ", ".join(sorted(self.names.values()))))

    def detect(self, rgb, name=None):
        """``[(name, score, (x1, y1, x2, y2)), ...]``, best first."""
        classes = None if name is None else [self.class_id(name)]
        r = self.model(np.ascontiguousarray(rgb[:, :, ::-1]), classes=classes,
                       conf=self.min_score, verbose=False)[0]
        out = [(self.names[int(c)], float(s), tuple(float(v) for v in xyxy))
               for c, s, xyxy in zip(r.boxes.cls.cpu().numpy(), r.boxes.conf.cpu().numpy(),
                                     r.boxes.xyxy.cpu().numpy())]
        return sorted(out, key=lambda d: -d[1])


# ------------------------------------------------------------------ geometry

def camera_pose(robot):
    """``(R, t)``: the colour camera's optical frame in the base frame.

    Set ``robot.angle_vector(ri.angle_vector())`` first, so the head angles
    are the measured ones. Optical frame: x right, y down, z forward.
    """
    c = robot.camera_color_optical_frame.copy_worldcoords()
    return c.worldrot(), c.worldpos()


def pixel_ray(K, u, v):
    """Unit ray through pixel (u, v) in the optical frame."""
    d = np.array([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], 1.0])
    return d / np.linalg.norm(d)


def pixel_on_floor(robot, K, u, v, floor_z):
    """Where the ray through (u, v) meets the floor, in the base frame.

    For things standing on the floor this gives their distance from the
    colour image alone, at any range: the D405's depth stops at 0.5 m.

    Parameters
    ----------
    floor_z : float
        Height of the floor in the base frame [m] (negative: base_link is
        above the floor).

    Returns
    -------
    numpy.ndarray or None
        ``(x, y, z)`` [m], or None if the ray does not go down to the floor.
    """
    return pixel_on_plane(robot, K, u, v, floor_z)


def pixel_on_plane(robot, K, u, v, z):
    """Where the ray through (u, v) meets the horizontal plane at height z.

    The floor (feet, things standing on it) or any known height, e.g. a
    person's head. None if the ray never gets there (it points away).
    """
    R, t = camera_pose(robot)
    d = R.dot(pixel_ray(K, u, v))
    if abs(d[2]) < 1e-6:
        return None
    s = (z - t[2]) / d[2]
    if s <= 0:
        return None
    return t + s * d


# ------------------------------------------------------------------ AprilTag

@dataclass
class Tag:
    """One AprilTag (36h11) in the base frame.

    Attributes
    ----------
    id : int
    position : numpy.ndarray
        Centre of the tag [m].
    normal : numpy.ndarray
        Unit vector out of the tag's face (towards whoever can read it).
    rotation : numpy.ndarray
        ``(3, 3)`` tag frame in the base frame (OpenCV's aruco convention:
        z points out of the face, towards the camera).
    corners : numpy.ndarray
        ``(4, 2)`` image corners [px].
    """

    id: int
    position: np.ndarray
    normal: np.ndarray
    rotation: np.ndarray
    corners: np.ndarray


class TagDetector:
    """AprilTag 36h11 with OpenCV's aruco module, pose by IPPE (colour only).

    Parameters
    ----------
    size : float or dict
        Edge of the tag's black square [m]: one for every tag, or per id
        (``{id: edge}``, e.g. :data:`DEMO_TAG_SIZES`; a tag whose id is not
        in it is skipped -- its distance cannot be known).
    upscale : int
        Look for tags in the image enlarged this many times (corners are
        scaled back). For small tags: the demo cube's 3.6 cm tags, ~50 px
        across in the 640x480 image from 0.25 m, were not found at 1 and
        were at 2 (OpenCV 4, Gazebo image).
    """

    def __init__(self, size, upscale=1):
        import cv2
        self._cv2 = cv2
        self._upscale = int(upscale)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        params = cv2.aruco.DetectorParameters()
        # Wider range of threshold windows than the default (3-23 px), and
        # markers allowed close together (a cube's faces meet at an edge):
        # with these the demo cube's front tag was found in 9-10/10 frames
        # instead of 2-5/10 at 0.25 m (Gazebo, 640x480, upscale 2).
        params.adaptiveThreshWinSizeMin = 3
        params.adaptiveThreshWinSizeMax = 53
        params.adaptiveThreshWinSizeStep = 4
        params.minMarkerDistanceRate = 0.01
        # Read each bit from an 8x8 px patch, not 4x4, and away from the
        # cell edges: a cube's tag seen at a slant has blurred cell edges,
        # and with the defaults it was found in 0/10 images at 0.2 m
        # (with these, the front and top tags both).
        params.perspectiveRemovePixelPerCell = 8
        params.perspectiveRemoveIgnoredMarginPerCell = 0.2
        self._detector = cv2.aruco.ArucoDetector(dictionary, params)
        self._size = size

    def _corners_3d(self, tag_id):
        size = self._size.get(tag_id) if isinstance(self._size, dict) else self._size
        if size is None:
            return None
        h = size / 2.0
        return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float32)

    def detect(self, rgb, K, robot):
        """Tags in the image, in the robot's base frame.

        Parameters
        ----------
        rgb : numpy.ndarray
        K : numpy.ndarray
            ``ri.camera_info()[0]``.
        robot : skrobot model
            With the measured joint angles (for the camera pose).

        Returns
        -------
        list of Tag
        """
        cv2 = self._cv2
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        if self._upscale > 1:
            gray = cv2.resize(gray, None, fx=self._upscale, fy=self._upscale,
                              interpolation=cv2.INTER_CUBIC)
        corners, ids, _ = self._detector.detectMarkers(gray)
        if ids is None:
            return []
        # Pixel centres: x_up = (x + 0.5) * k - 0.5.
        corners = [(c + 0.5) / self._upscale - 0.5 for c in corners]
        R_cam, t_cam = camera_pose(robot)
        tags = []
        for c, tag_id in zip(corners, ids.ravel()):
            obj = self._corners_3d(int(tag_id))
            if obj is None:
                continue
            ok, rvec, tvec = cv2.solvePnP(obj, c[0], K, None,
                                          flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue
            R_tag, _ = cv2.Rodrigues(rvec)
            rot = R_cam.dot(R_tag)
            pos = t_cam + R_cam.dot(tvec.ravel())
            tags.append(Tag(int(tag_id), pos, rot[:, 2], rot, c[0]))
        return tags


# ------------------------------------------------------------------ robot helpers

#: Height of the floor in base_link [m]: the bottom of the wheels, from the
#: CAD (jedy_skrobot.interface.FLOOR_Z).
FLOOR_Z = -0.11625
#: AprilTag edge used by the demo stage (jedy_bringup TAG_SIZE) [m].
DEMO_TAG_SIZE = 0.08

# ------------------------------------------------------------------ tag cube

#: The demo stage's graspable cube (jedy_bringup TAG_CUBE): edge [m], tag
#: edge [m], and per tag id the face it is on, as (outward normal, the tag's
#: up) in the cube's frame -- jedy_bringup CUBE_FACES with its ids.
TAG_CUBE_EDGE = 0.045
TAG_CUBE_TAG = 0.036
#: Tag edge per id on the demo stage: tag 0 on the stand, 1-6 on the cube.
DEMO_TAG_SIZES = {0: DEMO_TAG_SIZE, **{i: TAG_CUBE_TAG for i in range(1, 7)}}
TAG_CUBE_FACES = {
    1: ((-1, 0, 0), (0, 0, 1)), 3: ((1, 0, 0), (0, 0, 1)),
    4: ((0, -1, 0), (0, 0, 1)), 2: ((0, 1, 0), (0, 0, 1)),
    5: ((0, 0, 1), (1, 0, 0)), 6: ((0, 0, -1), (1, 0, 0)),
}


@dataclass
class Cube:
    """The tag cube in the base frame, from whichever of its faces are seen.

    Attributes
    ----------
    position : numpy.ndarray
        Centre [m].
    rotation : numpy.ndarray
        ``(3, 3)`` the cube's frame (its x, y, z axes as columns).
    tags : list of Tag
        The face tags it was computed from.
    """

    position: np.ndarray
    rotation: np.ndarray
    tags: list


def _face_rotation(tag_id):
    """The tag frame (x right, y up, z out) in the cube frame."""
    normal, up = (np.array(v, float) for v in TAG_CUBE_FACES[tag_id])
    return np.column_stack([np.cross(up, normal), up, normal])


def cube_pose(tags, edge=TAG_CUBE_EDGE):
    """Where the cube is, from the face tags among ``tags``.

    Each face tag alone gives the whole pose: its centre is half an edge out
    from the cube's, along its normal, and its id says which face it is, so
    its rotation gives the cube's. With two or three faces in view the
    centres are averaged and the rotation is taken from the tag seen most
    squarely (IPPE's orientation is least reliable for an oblique tag).

    Returns
    -------
    Cube or None
        None if no face tag of the cube is in ``tags``.
    """
    faces = [t for t in tags if t.id in TAG_CUBE_FACES]
    if not faces:
        return None
    centres = [t.position - edge / 2.0 * t.normal for t in faces]

    def squareness(t):
        view = t.position / np.linalg.norm(t.position)   # from base, roughly the camera
        return -abs(np.dot(view, t.normal))
    best = min(faces, key=squareness)
    rotation = best.rotation.dot(_face_rotation(best.id).T)
    return Cube(np.mean(centres, axis=0), rotation, faces)


def set_head(robot, ri, pan_deg=0.0, tilt_deg=0.0, seconds=1.0):
    """Point the head (tilt positive = look down) and wait."""
    import math
    robot.angle_vector(ri.angle_vector())
    robot.head_joint0.joint_angle(math.radians(pan_deg))
    robot.head_joint1.joint_angle(math.radians(tilt_deg))
    ri.angle_vector(robot.angle_vector(), seconds, controller_type='head_controller')
    ri.wait_interpolation('head_controller')
    robot.angle_vector(ri.angle_vector())


def say(node, text):
    """Publish ``text`` to /robotsound_jp/text, without waiting.

    VOICEVOX says it (the same voice as ``ri.say``), if running::

        ros2 launch voicevox voicevox_texttospeech.launch.xml
    """
    from std_msgs.msg import String
    if not hasattr(node, '_jedy_say_pub'):
        node._jedy_say_pub = node.create_publisher(String, '/robotsound_jp/text', 10)
    node._jedy_say_pub.publish(String(data=text))
    print('say:', text)


def solve_ik(robot, arm, point, extra_seeds=(), rotation=None):
    """IK for ``arm`` to ``point``, trying several start poses.

    Position only, unless ``rotation`` (a ``(3, 3)`` matrix for the hand's
    ``end_coords``: x = the direction the fingers point, y = the axis they
    close along) is given.

    Inverse kinematics is a local search: from one start pose it may miss a
    target that is reachable from another (measured: a point 1.8 cm from a
    solved one failed from the reset pose). So it tries, in order, the arm's
    current pose in the model, the reset pose, the driving pose (jedy_skrobot
    TUCK_POSE) and ``extra_seeds`` -- each a ``{joint name: rad}`` dict -- and
    keeps the first solution. Current first: a chain of nearby targets (in
    front of an object, then at it) then gets nearby solutions, not a jump
    to another arm configuration between them. The rest of the body is left as it is.

    Returns
    -------
    bool
        True if solved; the model then holds the solution. False leaves the
        arm where it was.
    """
    import math
    from skrobot.coordinates import Coordinates
    from jedy_skrobot.interface import TUCK_POSE, _MIRRORED
    joints = arm.joint_list
    current = {j.name: j.joint_angle() for j in joints}
    saved = robot.angle_vector().copy()
    robot.reset_pose()
    reset = {j.name: j.joint_angle() for j in joints}
    robot.angle_vector(saved)
    side = joints[0].name.split('_')[0]
    sign = 1.0 if side == 'rarm' else -1.0
    tuck = {}
    for j in joints:
        key = j.name.split('_', 1)[1]
        deg = TUCK_POSE.get(key, 0.0)
        tuck[j.name] = math.radians(sign * deg if key in _MIRRORED else deg)
    for seed in (current, reset, tuck) + tuple(extra_seeds):
        for j in joints:
            j.joint_angle(seed.get(j.name, current[j.name]))
        if rotation is None:
            ok = arm.inverse_kinematics(Coordinates(pos=point), rotation_axis=False)
        else:
            ok = arm.inverse_kinematics(Coordinates(pos=point, rot=rotation),
                                        rotation_axis=True)
        if ok is not False and ok is not None:
            return True
    for j in joints:
        j.joint_angle(current[j.name])
    return False
