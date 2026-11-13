"""Ask the course's VLM (Gemini on Vertex AI) about camera images.

The course runs an OpenAI-compatible gateway (the AI TA's server). Students
use it with the same key as the AI TA, so there is nothing to set up beyond
``ta login``: the key is read from ``AITA_BASE_URL`` / ``AITA_KEY`` in the
environment, or from the file ``ta login`` writes (``~/.config/ai-ta/env``)::

    from jedy_skrobot import connect
    from jedy_skrobot.vlm import VlmClient

    robot, ri = connect()
    vlm = VlmClient()                       # model 'vlm-er' or 'vlm-gemini'
    rgb, _ = ri.image()
    print(vlm.ask('何が見える? 1 文で答えて', images=[rgb]))

The server's limits, per student: one VLM request per second, $2 of VLM a
day, at most 2 images per request (JPEG/PNG ``data:`` URLs, 4 MB each). Every
request, images included, is saved and seen by the course staff.
:class:`VlmClient` waits so that its own requests are at least
:data:`MIN_INTERVAL_S` apart; a refusal raises :class:`VlmRefused` with the
server's message, which a program driving the robot must treat as "stop".

Only the Python standard library, numpy and OpenCV, all in jedy_skrobot's
venv.
"""

import base64
import json
import os
import socket
import time
import urllib.error
import urllib.request

import cv2
import numpy as np

#: Models of the gateway that take images.
MODELS = ('vlm-er', 'vlm-gemini')
#: Seconds between two of our requests. The server accepts one a second per
#: student and a refused request does not reset its clock, so keep a margin.
MIN_INTERVAL_S = 1.1
#: The server's limit on images in one request.
MAX_IMAGES = 2
#: Seconds to wait for one answer.
TIMEOUT_S = 60.0


class VlmRefused(RuntimeError):
    """The gateway did not answer the question.

    Parameters
    ----------
    kind : str
        ``'rate'`` (more than one request a second; wait and try again),
        ``'budget'`` (today's VLM budget is used up), ``'key'`` (no key, or
        the key is not accepted for this model), ``'request'`` (the request
        itself was refused), ``'server'`` (the server or the network failed;
        may work again later).
    message : str
        For the student, in Japanese, with the server's own words.
    status : int or None
        The HTTP status, if there was an answer.
    """

    def __init__(self, kind, message, status=None):
        super(VlmRefused, self).__init__(message)
        self.kind = kind
        self.status = status

    @property
    def retryable(self):
        """Whether waiting and asking again can help."""
        return self.kind in ('rate', 'server')


def env_file():
    """Path of the file ``ta login`` writes the key to."""
    config = os.environ.get('AITA_CONFIG_DIR') or os.path.expanduser(
        '~/.config/ai-ta')
    return os.path.join(config, 'env')


def load_credentials():
    """The gateway's URL and the student's key.

    ``AITA_BASE_URL`` / ``AITA_KEY`` from the environment win; otherwise
    they are read from :func:`env_file` (``KEY=value`` lines, as ``ta login``
    writes them).

    Returns
    -------
    tuple of str
        ``(base_url, key)``.

    Raises
    ------
    VlmRefused
        ``kind='key'`` if there is no key.
    """
    values = {}
    path = env_file()
    if os.path.exists(path):
        with open(path) as f:
            for line in f:
                line = line.strip()
                if line.startswith('export '):
                    line = line[len('export '):]
                if '=' not in line or line.startswith('#'):
                    continue
                name, value = line.split('=', 1)
                values[name.strip()] = value.strip().strip('\'"')
    base_url = os.environ.get('AITA_BASE_URL') or values.get('AITA_BASE_URL')
    key = os.environ.get('AITA_KEY') or values.get('AITA_KEY')
    if not base_url or not key:
        raise VlmRefused(
            'key',
            'VLM のキーがありません ({} に AITA_BASE_URL / AITA_KEY が無い)。'
            'AI TA と同じキーを使うので，ta login を実行してください。'.format(path))
    return base_url.rstrip('/'), key


def image_data_url(rgb, max_width=640, quality=80):
    """An RGB image as a JPEG ``data:`` URL, shrunk to ``max_width``.

    Parameters
    ----------
    rgb : numpy.ndarray
        ``(H, W, 3)`` uint8 RGB, as from ``ri.image()``.
    max_width : int
        Wider images are scaled down (fewer tokens, smaller upload).
    quality : int
        JPEG quality, 0-100.

    Returns
    -------
    str
    """
    bgr = cv2.cvtColor(np.asarray(rgb, dtype=np.uint8), cv2.COLOR_RGB2BGR)
    height, width = bgr.shape[:2]
    if width > max_width:
        bgr = cv2.resize(bgr, (max_width, int(round(height * max_width / width))),
                         interpolation=cv2.INTER_AREA)
    ok, jpeg = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise ValueError('could not encode the image as JPEG')
    return 'data:image/jpeg;base64,' + base64.b64encode(jpeg.tobytes()).decode()


def _refusal(status, body):
    """VlmRefused for an HTTP error answer, in Japanese."""
    try:
        detail = json.loads(body)
        error = detail.get('error', detail)
        message = error.get('message') if isinstance(error, dict) else None
        message = message or detail.get('detail') or body
    except (ValueError, AttributeError):
        message = body
    message = str(message).strip()[:500]
    lower = message.lower()
    if status == 429 and '予算' not in message and 'budget' not in lower:
        return VlmRefused('rate', 'VLM サーバーが断りました (HTTP 429，回数の制限): '
                          + message, status)
    if '予算' in message or 'budget' in lower:
        return VlmRefused('budget', 'VLM の予算を使い切りました (HTTP {})。'
                          'サーバーの返事: {}'.format(status, message),
                          status)
    if status == 403 and ('同意' in message or 'consent' in lower):
        return VlmRefused('key', 'このキーはまだ VLM (カメラ画像を AI に送る機能) に'
                          '同意していません '
                          '(HTTP 403)。ta login をやり直し，VLM の利用 (送った画像が記録され'
                          '教員・TA が見ること) に同意してください。'
                          'サーバーの返事: {}'.format(message), status)
    if status in (401, 403):
        return VlmRefused('key', 'VLM サーバーがこのキーを受け付けません (HTTP {})。'
                          'ta login をやり直すか，TA に聞いてください。'
                          'サーバーの返事: {}'.format(status, message), status)
    if status >= 500:
        return VlmRefused('server', 'VLM サーバーのエラーです (HTTP {}): {}'.format(
            status, message), status)
    return VlmRefused('request', 'VLM サーバーが問い合わせを断りました (HTTP {}): {}'
                      .format(status, message), status)


class VlmClient(object):
    """Questions with images to the course's VLM gateway.

    Parameters
    ----------
    model : str
        ``'vlm-er'`` (Gemini Robotics-ER, made for robots: where things are,
        which way to go) or ``'vlm-gemini'`` (Gemini).
    base_url, key : str, optional
        Default: :func:`load_credentials`.
    reasoning_effort : str or None
        ``'low'``, ``'medium'``, ``'high'``; None leaves the model's default.
        Less thinking answers sooner and costs less.
    """

    def __init__(self, model='vlm-er', base_url=None, key=None,
                 reasoning_effort='low'):
        if base_url is None or key is None:
            base_url, key = load_credentials()
        self.model = model
        self.base_url = base_url
        self._key = key
        self.reasoning_effort = reasoning_effort
        self._last_sent = None
        #: Cost of the last answer and of all answers so far [USD], as the
        #: server reports them.
        self.last_cost = 0.0
        self.total_cost = 0.0
        #: Seconds the last answer took.
        self.last_latency = 0.0

    def ask(self, text, images=(), system=None, max_tokens=2048):
        """Ask ``text`` about ``images``; return the answer's text.

        Waits first if the previous request was less than
        :data:`MIN_INTERVAL_S` ago.

        Parameters
        ----------
        text : str
        images : sequence of numpy.ndarray
            RGB images (at most :data:`MAX_IMAGES`).
        system : str, optional
            Instructions sent as the system message.
        max_tokens : int
            Upper bound on the answer, thinking included: with too small a
            bound the model thinks and is cut off before it answers.

        Returns
        -------
        str

        Raises
        ------
        VlmRefused
            The server refused or failed; see ``kind``.
        """
        if len(images) > MAX_IMAGES:
            raise ValueError('the server takes at most {} images per request'
                             .format(MAX_IMAGES))
        content = [{'type': 'text', 'text': text}]
        for rgb in images:
            content.append({'type': 'image_url',
                            'image_url': {'url': image_data_url(rgb)}})
        messages = [{'role': 'user', 'content': content}]
        if system:
            messages.insert(0, {'role': 'system', 'content': system})
        body = {'model': self.model, 'messages': messages,
                'max_tokens': max_tokens}
        if self.reasoning_effort:
            body['reasoning_effort'] = self.reasoning_effort
        request = urllib.request.Request(
            self.base_url + '/v1/chat/completions',
            data=json.dumps(body).encode(),
            headers={'Authorization': 'Bearer ' + self._key,
                     'Content-Type': 'application/json'})
        if self._last_sent is not None:
            wait = MIN_INTERVAL_S - (time.monotonic() - self._last_sent)
            if wait > 0:
                time.sleep(wait)
        self._last_sent = time.monotonic()
        try:
            with urllib.request.urlopen(request, timeout=TIMEOUT_S) as response:
                answer = json.loads(response.read())
                cost = response.headers.get('x-litellm-response-cost')
        except urllib.error.HTTPError as exc:
            raise _refusal(exc.code, exc.read().decode(errors='replace'))
        except (urllib.error.URLError, socket.timeout, ConnectionError) as exc:
            raise VlmRefused('server', 'VLM サーバーにつながりません ({}): {}'.format(
                self.base_url, getattr(exc, 'reason', exc)))
        self.last_latency = time.monotonic() - self._last_sent
        try:
            self.last_cost = float(cost) if cost else 0.0
        except ValueError:
            self.last_cost = 0.0
        self.total_cost += self.last_cost
        choice = (answer.get('choices') or [{}])[0]
        reply = (choice.get('message') or {}).get('content') or ''
        if not reply and choice.get('finish_reason') == 'length':
            raise VlmRefused('request', 'VLM の答えが max_tokens ({}) で切れました'
                             .format(max_tokens))
        return reply
