"""Which VLM backend ``ri.ask`` / ``ri.point`` talk to, against a local mock server.

GOOGLE_API_KEY set: Google AI Studio's OpenAI-compatible API (here a server on
127.0.0.1, through JEDY_VLM_GOOGLE_BASE_URL). Unset: the AI TA server, with the
same request as before. No real key and no network are used.

Run with the jedy_skrobot venv (interface.py imports rclpy and skrobot)::

    $(ros2 pkg prefix jedy_skrobot)/share/jedy_skrobot/venv/bin/python -m pytest test/
"""

import http.server
import json
import os
import sys
import threading
import types

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from jedy_skrobot import interface  # noqa: E402
from jedy_skrobot.interface import JedyInterface  # noqa: E402
from jedy_skrobot.interface import VlmRefused  # noqa: E402


class _Handler(http.server.BaseHTTPRequestHandler):
    """Records each request; answers with the server's next ``(status, body, headers)``."""

    def do_POST(self):
        length = int(self.headers.get('Content-Length', 0))
        self.server.requests.append({
            'path': self.path,
            'headers': dict(self.headers),
            'body': json.loads(self.rfile.read(length)),
        })
        status, body, headers = self.server.answer
        data = json.dumps(body).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(data)))
        for name, value in headers.items():
            self.send_header(name, value)
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, *args):
        pass


def _ok(text):
    return {'choices': [{'message': {'role': 'assistant', 'content': text},
                         'finish_reason': 'stop'}]}


@pytest.fixture
def server():
    srv = http.server.HTTPServer(('127.0.0.1', 0), _Handler)
    srv.requests = []
    srv.answer = (200, _ok('hello'), {'x-litellm-response-cost': '0.0123'})
    thread = threading.Thread(target=srv.serve_forever, daemon=True)
    thread.start()
    yield srv
    srv.shutdown()
    srv.server_close()


@pytest.fixture
def env(monkeypatch, tmp_path):
    """No key of either kind unless a test sets one."""
    for name in ('GOOGLE_API_KEY', 'JEDY_VLM_GOOGLE_BASE_URL', 'JEDY_VLM_GEMINI_MODEL',
                 'JEDY_VLM_ER_MODEL', 'AITA_BASE_URL', 'AITA_KEY'):
        monkeypatch.delenv(name, raising=False)
    monkeypatch.setattr(interface, 'AITA_ENV_FILE', str(tmp_path / 'no-env'))
    monkeypatch.setattr(interface, '_vlm_backend_logged', False)
    return monkeypatch


class _Logger:
    def __init__(self):
        self.lines = []

    def info(self, text):
        self.lines.append(text)


def _fake_ri():
    """Just what ``JedyInterface._vlm_chat`` uses of ``self``: no ROS node."""
    logger = _Logger()
    return types.SimpleNamespace(
        _vlm_lock=threading.Lock(), _vlm_sent=None, vlm_last_wait=0.0,
        vlm_last_latency=0.0, vlm_last_cost=0.0, vlm_total_cost=0.0,
        get_logger=lambda: logger, logger=logger)


def _image_messages():
    url = interface._jpeg_data_url(np.zeros((48, 64, 3), np.uint8), 640)
    return url, [{'role': 'user', 'content': [
        {'type': 'text', 'text': 'what is this?'},
        {'type': 'image_url', 'image_url': {'url': url}}]}]


def _use_google(env, server):
    env.setenv('GOOGLE_API_KEY', 'test-google-key')
    env.setenv('JEDY_VLM_GOOGLE_BASE_URL',
               'http://127.0.0.1:{}/v1beta/openai/'.format(server.server_port))


def test_google_request(env, server):
    _use_google(env, server)
    ri = _fake_ri()
    url, messages = _image_messages()
    schema = {'type': 'json_schema', 'json_schema': {'name': 'answer', 'schema': {}}}
    text = JedyInterface._vlm_chat(ri, 'vlm-gemini', messages, 10.0, 'low',
                                   response_format=schema)
    assert text == 'hello'
    (req,) = server.requests
    assert req['path'] == '/v1beta/openai/chat/completions'
    assert req['headers']['Authorization'] == 'Bearer test-google-key'
    body = req['body']
    assert body['model'] == 'gemini-3.8-flash'
    assert body['messages'][0]['content'][1]['image_url']['url'] == url
    assert url.startswith('data:image/jpeg;base64,')
    assert body['reasoning_effort'] == 'low'
    assert body['response_format'] == schema
    assert body['max_tokens'] == interface.VLM_MAX_TOKENS
    # The student's own key: no LiteLLM cost, even if a header said one.
    assert ri.vlm_last_cost == 0.0 and ri.vlm_total_cost == 0.0
    assert ri.logger.lines == ['VLM: Google AI Studio (GOOGLE_API_KEY)']
    JedyInterface._vlm_chat(ri, 'vlm-gemini', messages, 10.0, 'low')
    assert len(ri.logger.lines) == 1   # once per process


def test_google_er_model_reasoning_none_and_override(env, server):
    _use_google(env, server)
    _, messages = _image_messages()
    JedyInterface._vlm_chat(_fake_ri(), 'vlm-er', messages, 10.0, 'none')
    body = server.requests[-1]['body']
    assert body['model'] == 'gemini-robotics-er-2-preview'
    assert body['reasoning_effort'] == 'minimal'   # Gemini 3 has no 'none'
    assert 'response_format' not in body
    env.setenv('JEDY_VLM_ER_MODEL', 'gemini-robotics-er-9')
    env.setenv('JEDY_VLM_GEMINI_MODEL', 'gemini-9-flash')
    JedyInterface._vlm_chat(_fake_ri(), 'vlm-er', messages, 10.0, 'high')
    assert server.requests[-1]['body']['model'] == 'gemini-robotics-er-9'
    JedyInterface._vlm_chat(_fake_ri(), 'vlm-gemini', messages, 10.0, 'high')
    assert server.requests[-1]['body']['model'] == 'gemini-9-flash'


@pytest.mark.parametrize('status, message, kind, words', [
    (401, 'Request had invalid authentication credentials.', 'key', 'GOOGLE_API_KEY'),
    (400, 'API key not valid. Please pass a valid API key.', 'key', 'tips/google-ai-studio'),
    (403, 'Permission denied.', 'key', 'GOOGLE_API_KEY'),
    (429, 'Resource has been exhausted (e.g. check quota).', 'rate', '無料枠'),
    (404, 'models/gemini-robotics-er-2-preview is not found', 'request', 'JEDY_VLM_ER_MODEL'),
    (400, 'Invalid JSON payload received.', 'request', 'Invalid JSON payload'),
    (503, 'The model is overloaded.', 'server', 'Google AI Studio'),
])
def test_google_errors(env, server, status, message, kind, words):
    _use_google(env, server)
    # Google's OpenAI-compatible API sometimes wraps the error in a list.
    server.answer = (status, [{'error': {'code': status, 'message': message}}], {})
    _, messages = _image_messages()
    with pytest.raises(VlmRefused) as info:
        JedyInterface._vlm_chat(_fake_ri(), 'vlm-er', messages, 10.0, 'none')
    exc = info.value
    assert exc.kind == kind and exc.status == status
    assert words in str(exc)
    assert message in str(exc)
    assert 'ta login' not in str(exc)
    if status == 404:
        assert 'gemini-robotics-er-2-preview' in str(exc)


def test_aita_request_unchanged(env, server):
    """Without GOOGLE_API_KEY: the request the AI TA server always got."""
    env.setenv('GOOGLE_API_KEY', '')   # set but empty is "not set"
    env.setenv('AITA_BASE_URL', 'http://127.0.0.1:{}/'.format(server.server_port))
    env.setenv('AITA_KEY', 'sk-aita')
    ri = _fake_ri()
    _, messages = _image_messages()
    JedyInterface._vlm_chat(ri, 'vlm-gemini', messages, 10.0, 'none')
    (req,) = server.requests
    assert req['path'] == '/v1/chat/completions'
    assert req['headers']['Authorization'] == 'Bearer sk-aita'
    assert req['body'] == {'model': 'vlm-gemini', 'messages': messages,
                           'max_tokens': interface.VLM_MAX_TOKENS,
                           'reasoning_effort': 'none'}
    assert ri.vlm_last_cost == pytest.approx(0.0123)
    assert ri.logger.lines == ['VLM: AI TA server']


def test_aita_errors_unchanged(env, server):
    env.setenv('AITA_BASE_URL', 'http://127.0.0.1:{}'.format(server.server_port))
    env.setenv('AITA_KEY', 'sk-aita')
    server.answer = (403, {'error': {'message': 'consent required'}}, {})
    _, messages = _image_messages()
    with pytest.raises(VlmRefused) as info:
        JedyInterface._vlm_chat(_fake_ri(), 'vlm-gemini', messages, 10.0, 'none')
    assert info.value.kind == 'key' and 'ta login' in str(info.value)


def test_no_key_at_all(env):
    with pytest.raises(VlmRefused) as info:
        interface.vlm_credentials()
    assert info.value.kind == 'key' and 'ta login' in str(info.value)
    assert 'GOOGLE_API_KEY' in str(info.value)   # the student's own key, too
    with pytest.raises(VlmRefused):
        interface.aita_credentials()   # the old name still works


def test_google_error_does_not_switch_to_aita(env, server):
    """Both keys present: GOOGLE_API_KEY wins, and its refusal is final."""
    _use_google(env, server)
    env.setenv('AITA_BASE_URL', 'http://127.0.0.1:{}'.format(server.server_port))
    env.setenv('AITA_KEY', 'sk-aita')
    server.answer = (403, {'error': {'message': 'Permission denied.'}}, {})
    _, messages = _image_messages()
    with pytest.raises(VlmRefused):
        JedyInterface._vlm_chat(_fake_ri(), 'vlm-gemini', messages, 10.0, 'none')
    assert [r['path'] for r in server.requests] == ['/v1beta/openai/chat/completions']
    assert interface.vlm_credentials()[0] == 'google'


def test_google_flash_reasoning_none_is_low(env, server):
    # gemini-3.8-flash answers 400 "Thinking level MINIMAL is not supported"
    # (free-tier key, 2026-11-14), so ask's default 'none' goes as 'low'.
    _use_google(env, server)
    _, messages = _image_messages()
    for given in ('none', 'minimal'):
        JedyInterface._vlm_chat(_fake_ri(), 'vlm-gemini', messages, 10.0, given)
        assert server.requests[-1]['body']['reasoning_effort'] == 'low'
