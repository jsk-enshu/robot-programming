"""Recognition by words and by hands: what the recognition nodes run.

:class:`WordDetector` finds objects named by words (YOLOE, an open-vocabulary
YOLO: any word, not only YOLO's 80 classes), :class:`GestureReader` reads a
hand gesture (MediaPipe Gesture Recognizer). Both take the RGB images that
``ri.image()`` returns and run on the CPU.

The nodes (``object_detection_node``, ``gesture_node``) wrap them, and
``ri.what_do_you_see()``, ``ri.sees()``, ``ri.gesture()`` and ``ri.janken()``
call the nodes, so a program using ``ri`` needs none of the packages here.

Japanese words are translated to the English the detector understands with
``config/vocabulary.yaml``.
"""

import os

import numpy as np

from jedy_vision import model_path

#: MediaPipe's gesture names in Japanese. ``None`` (no gesture) is left out.
GESTURE_JA = {
    'Closed_Fist': 'グー',
    'Victory': 'チョキ',
    'Open_Palm': 'パー',
    'Pointing_Up': '指さし',
    'Thumb_Up': 'いいね',
    'Thumb_Down': 'よくない',
    'ILoveYou': 'アイラブユー',
}
#: The gestures that are janken hands.
JANKEN = ('グー', 'チョキ', 'パー')

#: YOLOE model and its text encoder (they come as a pair: yoloe-26 was trained
#: with MobileCLIP2). Downloaded at build time, see CMakeLists.txt.
YOLOE_WEIGHTS = 'yoloe-26s-seg.pt'
YOLOE_TEXT_ENCODER = 'mobileclip2_b.ts'
GESTURE_MODEL = 'gesture_recognizer.task'


def vocabulary_path():
    """``share/jedy_vision/config/vocabulary.yaml``.

    Returns
    -------
    str
    """
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('jedy_vision'),
                        'config', 'vocabulary.yaml')


def load_vocabulary(path=None):
    """The Japanese -> English table.

    Parameters
    ----------
    path : str or None
        ``None``: :func:`vocabulary_path`.

    Returns
    -------
    dict
        ``{Japanese word: English word}``.
    """
    import yaml
    with open(path or vocabulary_path(), encoding='utf-8') as f:
        table = yaml.safe_load(f) or {}
    return {str(k): str(v) for k, v in table.items()}


def is_ascii(word):
    """Whether a word is plain ASCII (English), not Japanese."""
    return all(ord(c) < 128 for c in word)


class WordDetector:
    """Objects named by words, with YOLOE.

    Parameters
    ----------
    min_score : float
        Detections below this confidence are dropped.
    vocabulary : dict or None
        ``{Japanese: English}``; ``None`` loads ``config/vocabulary.yaml``.

    Notes
    -----
    Changing the words runs the text encoder (about 0.2 s on a laptop CPU),
    so it is done only when they change; one image then takes about 0.15 s.
    """

    def __init__(self, min_score=0.25, vocabulary=None):
        import torch
        from ultralytics import YOLOE
        from ultralytics.nn.text_model import MobileCLIPTS
        self.min_score = min_score
        self.vocabulary = load_vocabulary() if vocabulary is None else vocabulary
        self.model = YOLOE(model_path(YOLOE_WEIGHTS))
        # The text encoder from the build's copy. Left to itself, YOLOE
        # looks for it by bare file name in the current directory and in
        # Ultralytics' settings directory, and downloads it if it is in
        # neither -- at run time, while jedy_vision runs offline.
        self.model.model.clip_model = MobileCLIPTS(
            torch.device('cpu'), weight=model_path(YOLOE_TEXT_ENCODER))
        self.words = []
        self._to_word = {}

    def translate(self, word):
        """The English the detector is given for ``word``.

        Returns
        -------
        str

        Raises
        ------
        ValueError
            A Japanese word missing from the vocabulary: the detector's text
            encoder only knows English, and would silently find nothing.
        """
        if word in self.vocabulary:
            return self.vocabulary[word]
        if is_ascii(word):
            return word
        raise ValueError(
            '「{}」は対応表にありません. jedy_vision の config/vocabulary.yaml に '
            '「{}: <英語の名詞>」を足すか, 英語で指定してください'.format(word, word))

    def set_words(self, words):
        """Look for these words from now on.

        Parameters
        ----------
        words : list of str
            Japanese (in the vocabulary) or English.
        """
        words = list(dict.fromkeys(str(w) for w in words))
        if not words:
            raise ValueError('words is empty: say what to look for')
        if words == self.words:
            return
        english = [self.translate(w) for w in words]
        # Two words with the same English (お茶 and ペットボトル are both
        # "bottle") are one class for the detector; the first one names it.
        classes = list(dict.fromkeys(english))
        embeddings = self.model.model.get_text_pe(classes, cache_clip_model=True)
        self.model.set_classes(classes, embeddings)
        self._to_word = {}
        for w, e in zip(words, english):
            self._to_word.setdefault(e, w)
        self.words = words

    def detect(self, rgb):
        """The words' objects in an RGB image, most confident first.

        Parameters
        ----------
        rgb : numpy.ndarray
            ``(H, W, 3)`` uint8 RGB.

        Returns
        -------
        list of tuple
            ``(word, score, (x1, y1, x2, y2))``, the box in pixels.
        """
        if not self.words:
            raise ValueError('call set_words() first')
        # Ultralytics takes a numpy image as OpenCV's BGR.
        r = self.model.predict(np.ascontiguousarray(rgb[:, :, ::-1]),
                               conf=self.min_score, verbose=False)[0]
        out = []
        for c, s, xyxy in zip(r.boxes.cls.cpu().numpy(), r.boxes.conf.cpu().numpy(),
                              r.boxes.xyxy.cpu().numpy()):
            english = r.names[int(c)]
            out.append((self._to_word.get(english, english), float(s),
                        tuple(float(v) for v in xyxy)))
        return sorted(out, key=lambda d: -d[1])


class GestureReader:
    """A hand gesture in an image, with MediaPipe's Gesture Recognizer.

    Parameters
    ----------
    max_hands : int
        How many hands to look at; the most confident gesture is returned.
    """

    def __init__(self, max_hands=2):
        import mediapipe as mp
        from mediapipe.tasks.python import BaseOptions, vision
        self._mp = mp
        options = vision.GestureRecognizerOptions(
            base_options=BaseOptions(model_asset_path=model_path(GESTURE_MODEL)),
            running_mode=vision.RunningMode.IMAGE,
            num_hands=max_hands)
        self._recognizer = vision.GestureRecognizer.create_from_options(options)

    # ``with GestureReader() as reader:``, for the same reason as
    # PoseEstimator: MediaPipe's own cleanup at interpreter exit raises.
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    def read(self, rgb):
        """The most confident gesture of the hands in an RGB image.

        Parameters
        ----------
        rgb : numpy.ndarray
            ``(H, W, 3)`` uint8 RGB.

        Returns
        -------
        tuple or None
            ``(name, score, handedness)`` with MediaPipe's name (a key of
            :data:`GESTURE_JA`); None when there is no hand or no gesture.
        """
        image = self._mp.Image(image_format=self._mp.ImageFormat.SRGB,
                               data=np.ascontiguousarray(rgb))
        result = self._recognizer.recognize(image)
        best = None
        for gestures, hands in zip(result.gestures, result.handedness):
            g = gestures[0]
            if g.category_name not in GESTURE_JA:
                continue
            if best is None or g.score > best[1]:
                best = (g.category_name, float(g.score), hands[0].category_name)
        return best

    def close(self):
        """Release the recognizer. Call before the program exits."""
        if self._recognizer is not None:
            self._recognizer.close()
            self._recognizer = None


def decode_rgb(msg):
    """A colour sensor_msgs/Image as an RGB array, without cv_bridge.

    Parameters
    ----------
    msg : sensor_msgs.msg.Image
        ``rgb8``, ``bgr8``, ``rgba8``, ``bgra8``, ``mono8``, or
        ``yuv422_yuy2`` (what ``usb_cam`` publishes for a PC's web camera
        unless told ``pixel_format:=yuyv2rgb``).

    Returns
    -------
    numpy.ndarray
        ``(H, W, 3)`` uint8 RGB.
    """
    if msg.encoding in ('yuv422_yuy2', 'yuyv'):
        import cv2
        flat = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        yuyv = flat.reshape(msg.height, msg.step)[:, :msg.width * 2]
        return cv2.cvtColor(yuyv.reshape(msg.height, msg.width, 2), cv2.COLOR_YUV2RGB_YUY2)
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4,
                'mono8': 1}.get(msg.encoding)
    if channels is None:
        raise ValueError('image encoding {!r} is not a colour image'.format(
            msg.encoding))
    flat = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    rows = flat.reshape(msg.height, msg.step)[:, :msg.width * channels]
    img = rows.reshape(msg.height, msg.width, channels)
    if channels == 1:
        return np.repeat(img, 3, axis=2)
    if msg.encoding.startswith('bgr'):
        img = img[:, :, 2::-1]
    return np.ascontiguousarray(img[:, :, :3])


def encode_rgb(rgb, header):
    """An RGB array as a sensor_msgs/Image (``rgb8``)."""
    from sensor_msgs.msg import Image
    msg = Image()
    msg.header = header
    msg.height, msg.width = rgb.shape[:2]
    msg.encoding = 'rgb8'
    msg.step = msg.width * 3
    msg.data = np.ascontiguousarray(rgb).tobytes()
    return msg


def decode_depth(msg):
    """A depth sensor_msgs/Image in metres, without cv_bridge.

    Parameters
    ----------
    msg : sensor_msgs.msg.Image
        ``32FC1`` [m] (Gazebo) or ``16UC1`` [mm] (RealSense).

    Returns
    -------
    numpy.ndarray
        ``(H, W)`` float32 [m]; pixels without a reading are nan.
    """
    dtype = {'32FC1': np.float32, '16UC1': np.uint16}.get(msg.encoding)
    if dtype is None:
        raise ValueError('image encoding {!r} is not a depth image'.format(msg.encoding))
    item = np.dtype(dtype).itemsize
    flat = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    rows = flat.reshape(msg.height, msg.step)[:, :msg.width * item]
    depth = np.ascontiguousarray(rows).view(dtype).reshape(msg.height, msg.width)
    depth = depth.astype(np.float32)
    if dtype is np.uint16:
        depth = depth / 1000.0
    depth[~np.isfinite(depth) | (depth <= 0.0)] = np.nan
    return depth


def box_position(depth, box, K):
    """Where the object in ``box`` is, in the camera's optical frame [m].

    The box centre's pixel, back-projected to the median depth of the box's
    middle third (the edges of a box are mostly background).

    Parameters
    ----------
    depth : numpy.ndarray
        ``(H, W)`` [m], nan where there is no reading (:func:`decode_depth`).
    box : sequence of float
        ``x1, y1, x2, y2`` [px] in an image of the same size as ``depth``.
    K : numpy.ndarray
        ``(3, 3)`` camera matrix of that image.

    Returns
    -------
    tuple of float or None
        ``(x, y, z)`` (x right, y down, z forward), or None if no pixel of
        the middle third has a depth reading.
    """
    x1, y1, x2, y2 = (float(v) for v in box)
    h, w = depth.shape
    cx0, cx1 = int(x1 + (x2 - x1) / 3.0), int(x2 - (x2 - x1) / 3.0) + 1
    cy0, cy1 = int(y1 + (y2 - y1) / 3.0), int(y2 - (y2 - y1) / 3.0) + 1
    patch = depth[max(cy0, 0):min(cy1, h), max(cx0, 0):min(cx1, w)]
    values = patch[np.isfinite(patch)]
    if values.size == 0:
        return None
    z = float(np.median(values))
    u, v = 0.5 * (x1 + x2), 0.5 * (y1 + y2)
    return ((u - K[0, 2]) / K[0, 0] * z, (v - K[1, 2]) / K[1, 1] * z, z)


def draw_detections(rgb, detections, vocabulary, positions=None):
    """A copy of ``rgb`` with the detections' boxes and English names.

    OpenCV draws no Japanese, so a Japanese word is shown as its English.
    ``positions`` (one ``(x, y, z)`` or None per detection, camera frame)
    adds the distance, ``z`` [m], to each label.
    """
    import cv2
    out = rgb.copy()
    for i, (word, score, (x1, y1, x2, y2)) in enumerate(detections):
        name = word if is_ascii(word) else vocabulary.get(word, '?')
        p1, p2 = (int(x1), int(y1)), (int(x2), int(y2))
        label = '{} {:.2f}'.format(name, score)
        if positions is not None and positions[i] is not None:
            label += ' {:.2f} m'.format(positions[i][2])
        cv2.rectangle(out, p1, p2, (0, 255, 0), 2)
        cv2.putText(out, label, (p1[0], max(p1[1] - 5, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    return out
