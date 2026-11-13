"""Jedy talks and listens: Open JTalk speech, Whisper recognition, RViz bubbles.

RViz's own text markers cannot draw Japanese (they use a font texture with
ASCII glyphs only; a Japanese string comes out empty), so a speech bubble is
a small textured mesh instead: the text is drawn with Pillow and a Japanese
font into a PNG, and the mesh has the outline of the bubble (rounded box and
tail), so nothing outside it needs transparency. It is two-sided, readable
from the robot's front and back, and published as a MESH_RESOURCE marker.

Everything runs offline on the CPU; the model files are downloaded at build
time into ``share/jedy_speech/models``.
"""

import io
import math
import os
import shutil
import subprocess
import time
import wave

import numpy as np

#: Japanese font for the bubbles (Ubuntu package fonts-noto-cjk).
FONT_PATHS = ["/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
              "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"]
#: Characters that may not start a line (Japanese line-breaking rule).
NO_LINE_START = "、。，．,.!！?？)）」』】ーぁぃぅぇぉっゃゅょァィゥェォッャュョ"
SAMPLE_RATE = 16000


def share_dir():
    """``<prefix>/share/jedy_speech``."""
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory("jedy_speech")


def model_path(*parts):
    """Path under ``share/jedy_speech/models``, failing loudly if missing."""
    path = os.path.join(share_dir(), "models", *parts)
    if not os.path.exists(path):
        raise FileNotFoundError(
            "{} not found. It is downloaded by `colcon build --packages-select "
            "jedy_speech` (needs network once).".format(path))
    return path


# --------------------------------------------------------------- bubbles

def wrap_japanese(text, width):
    """Wrap text every ``width`` characters, keeping 、。 off line starts."""
    lines, line = [], ""
    for ch in text.replace("\n", ""):
        if len(line) >= width and ch not in NO_LINE_START:
            lines.append(line)
            line = ""
        line += ch
    if line:
        lines.append(line)
    return lines or [""]


def _font(size):
    from PIL import ImageFont
    for path in FONT_PATHS:
        if os.path.exists(path):
            return ImageFont.truetype(path, size, index=0)  # index 0: JP
    raise FileNotFoundError("no Japanese font; sudo apt install fonts-noto-cjk")


def _rounded_rect(x0, y0, x1, y1, r, steps=6):
    """Outline points of a rounded rectangle, image coordinates (y down)."""
    pts = []
    for cx, cy, a0 in ((x1 - r, y0 + r, -90), (x1 - r, y1 - r, 0),
                       (x0 + r, y1 - r, 90), (x0 + r, y0 + r, 180)):
        for k in range(steps + 1):
            a = math.radians(a0 + 90.0 * k / steps)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def render_bubble(text, png_path, width_chars=12, font_px=44,
                  fill=(255, 255, 255), border=(40, 40, 40), ink=(20, 20, 20),
                  credit=None):
    """Draw a speech bubble PNG and return its outline triangles.

    Returns
    -------
    size : tuple of int
        ``(width, height)`` of the image [px].
    triangles : list of tuple
        Triangles covering the bubble, in image pixels.
    """
    from PIL import Image, ImageDraw
    font = _font(font_px)
    lines = wrap_japanese(text, width_chars)
    pad, line_h, tail, radius = 26, font_px + 12, 34, 26
    small = _font(font_px // 2)
    credit_h = (font_px // 2 + 8) if credit else 0
    w = max([font.getbbox(line)[2] for line in lines]
            + ([small.getbbox(credit)[2]] if credit else [])) + 2 * pad
    h = line_h * len(lines) + 2 * pad + credit_h
    img = Image.new("RGB", (w, h + tail), fill)
    draw = ImageDraw.Draw(img)
    outline = _rounded_rect(3, 3, w - 4, h - 4, radius)
    draw.polygon(outline, fill=fill, outline=border)
    draw.line(outline + [outline[0]], fill=border, width=5)
    tx = w // 4
    tail_tri = [(tx, h - 8), (tx + 36, h - 8), (tx + 4, h + tail - 2)]
    draw.polygon(tail_tri, fill=fill)
    draw.line([tail_tri[0], tail_tri[2], tail_tri[1]], fill=border, width=5)
    for i, line in enumerate(lines):
        draw.text((pad, pad + i * line_h - 4), line, font=font, fill=ink)
    if credit:  # e.g. "VOICEVOX:ずんだもん", required by the voice's terms
        cw = small.getbbox(credit)[2]
        draw.text((w - pad - cw, h - pad - credit_h + 4), credit, font=small,
                  fill=(110, 110, 110))
    img.save(png_path)
    cx, cy = w / 2.0, h / 2.0
    tris = [((cx, cy), outline[i], outline[(i + 1) % len(outline)])
            for i in range(len(outline))]
    tris.append(tuple(tail_tri))
    return img.size, tris


def bubble_mesh(png_name, size, triangles, height_m):
    """Collada text of a two-sided mesh with the bubble's outline.

    The front faces +x (a viewer in front of the robot has the robot's -y on
    their right), the back faces -x, each with its own winding so the text
    reads left to right from both sides.
    """
    w_px, h_px = size
    s = height_m / h_px
    positions, uvs, normals, index = [], [], [(1, 0, 0), (-1, 0, 0)], []
    for side, sign in ((0, 1.0), (1, -1.0)):
        for tri in triangles:
            corners = []
            for px, py in tri:
                y = sign * (w_px / 2.0 - px) * s
                z = (h_px - py) * s
                positions.append((0.001 * sign, y, z))
                uvs.append((px / w_px, 1.0 - py / h_px))
                corners.append(len(positions) - 1)
            a, b, c = (np.array(positions[i]) for i in corners)
            # RViz shows the side whose triangle normal points AWAY from the
            # viewer here (found by rendering: the other way the text came
            # out mirrored), so the side seen from +x has a -x normal.
            if np.cross(b - a, c - a)[0] * sign > 0:
                corners[1], corners[2] = corners[2], corners[1]
            index.extend((i, side, i) for i in corners)
    def fl(rows):
        return " ".join("{:.5f}".format(v) for row in rows for v in row)
    return _COLLADA.format(
        png=png_name, npos=len(positions) * 3, nvert=len(positions),
        pos=fl(positions), nrm=fl(normals), nuv=len(uvs) * 2, uv=fl(uvs),
        ntri=len(index) // 3, p=" ".join("{} {} {}".format(*t) for t in index))


_COLLADA = """<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
 <asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
 <library_images><image id="img"><init_from>{png}</init_from></image></library_images>
 <library_effects><effect id="fx"><profile_COMMON>
  <newparam sid="surf"><surface type="2D"><init_from>img</init_from></surface></newparam>
  <newparam sid="samp"><sampler2D><source>surf</source></sampler2D></newparam>
  <technique sid="common"><lambert>
   <emission><color>0.85 0.85 0.85 1</color></emission>
   <diffuse><texture texture="samp" texcoord="UV0"/></diffuse>
  </lambert></technique></profile_COMMON></effect></library_effects>
 <library_materials><material id="mat"><instance_effect url="#fx"/></material></library_materials>
 <library_geometries><geometry id="g"><mesh>
  <source id="pos"><float_array id="pa" count="{npos}">{pos}</float_array><technique_common><accessor source="#pa" count="{nvert}" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
  <source id="nrm"><float_array id="na" count="6">{nrm}</float_array><technique_common><accessor source="#na" count="2" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
  <source id="uv"><float_array id="ua" count="{nuv}">{uv}</float_array><technique_common><accessor source="#ua" count="{nvert}" stride="2"><param name="S" type="float"/><param name="T" type="float"/></accessor></technique_common></source>
  <vertices id="v"><input semantic="POSITION" source="#pos"/></vertices>
  <triangles material="m" count="{ntri}">
   <input semantic="VERTEX" source="#v" offset="0"/><input semantic="NORMAL" source="#nrm" offset="1"/><input semantic="TEXCOORD" source="#uv" offset="2" set="0"/>
   <p>{p}</p></triangles></mesh></geometry></library_geometries>
 <library_visual_scenes><visual_scene id="s"><node id="n"><instance_geometry url="#g"><bind_material><technique_common>
  <instance_material symbol="m" target="#mat"><bind_vertex_input semantic="UV0" input_semantic="TEXCOORD" input_set="0"/></instance_material>
 </technique_common></bind_material></instance_geometry></node></visual_scene></library_visual_scenes>
 <scene><instance_visual_scene url="#s"/></scene>
</COLLADA>
"""


class BubbleMaker:
    """Write bubble meshes to files and build RViz markers for them.

    RViz caches a mesh by its URI, so every bubble gets a new file name; old
    ones are deleted (only the newest ``keep`` are kept).

    Parameters
    ----------
    directory : str or None
        Where to write; default ``~/.cache/jedy_speech/bubbles``.
    """

    def __init__(self, directory=None, keep=30):
        self.directory = directory or os.path.join(
            os.path.expanduser("~"), ".cache", "jedy_speech", "bubbles")
        os.makedirs(self.directory, exist_ok=True)
        self.keep = keep
        self.count = 0

    def make(self, text, height_m=0.22, **style):
        """Write one bubble; returns ``file://...dae``."""
        self.count += 1
        stem = "bubble_{}_{}".format(os.getpid(), self.count)
        png = stem + ".png"
        size, tris = render_bubble(text, os.path.join(self.directory, png), **style)
        path = os.path.join(self.directory, stem + ".dae")
        with open(path, "w") as f:
            f.write(bubble_mesh(png, size, tris, height_m))
        self._prune()
        return "file://" + path

    def _prune(self):
        files = sorted((os.path.join(self.directory, n) for n in os.listdir(self.directory)),
                       key=os.path.getmtime)
        for path in files[:-2 * self.keep]:
            os.remove(path)

    def marker(self, uri, frame_id, position, lifetime_s, ns="speech", marker_id=0):
        """A MESH_RESOURCE marker for a bubble made by :meth:`make`."""
        from builtin_interfaces.msg import Duration
        from visualization_msgs.msg import Marker
        m = Marker()
        m.header.frame_id = frame_id
        m.ns, m.id = ns, marker_id
        m.type, m.action = Marker.MESH_RESOURCE, Marker.ADD
        m.mesh_resource = uri
        m.mesh_use_embedded_materials = True
        m.pose.position.x, m.pose.position.y, m.pose.position.z = position
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 1.0
        secs = max(0.0, lifetime_s)
        m.lifetime = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        return m


# --------------------------------------------------------------- speech

class Speaker:
    """Japanese text to speech with Open JTalk, played with ``aplay``."""

    #: Text the bubble must show for this voice (None: no requirement).
    credit = None

    def __init__(self):
        os.environ.setdefault("OPEN_JTALK_DICT_DIR", model_path("open_jtalk_dic_utf_8-1.11"))
        import pyopenjtalk
        self._tts = pyopenjtalk.tts
        self._tts("あ")  # load the dictionary and the voice now

    def synthesize(self, text):
        """Return ``(int16 samples, sample_rate)``."""
        x, sr = self._tts(text)
        return np.clip(x, -32768, 32767).astype(np.int16), int(sr)

    @staticmethod
    def play(samples, sample_rate):
        """Play through the default sound device; raises if there is none."""
        if shutil.which("aplay") is None:
            raise RuntimeError("aplay not found (sudo apt install alsa-utils)")
        buf = io.BytesIO()
        with wave.open(buf, "wb") as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(sample_rate)
            w.writeframes(samples.tobytes())
        proc = subprocess.run(["aplay", "-q", "-"], input=buf.getvalue(),
                              stderr=subprocess.PIPE)
        if proc.returncode != 0:
            raise RuntimeError("aplay failed: {}".format(
                proc.stderr.decode(errors="replace").strip()))


#: Where `ros2 run jedy_speech setup_voicevox` puts VOICEVOX's downloads.
VOICEVOX_DIR = os.path.join(os.path.expanduser("~"), ".local", "share", "jedy_speech",
                            "voicevox")


class VoicevoxSpeaker(Speaker):
    """Text to speech with VOICEVOX CORE (e.g. ずんだもん), on the CPU.

    The voice models are NOT shipped with this package: each user downloads
    them with VOICEVOX's own downloader and accepts its terms
    (``ros2 run jedy_speech setup_voicevox``). Every voice requires a credit
    such as "VOICEVOX:ずんだもん" wherever its speech is used; :attr:`credit`
    holds it and the speaker node prints it in every bubble.

    Parameters
    ----------
    voice : str
        Character name, e.g. ``ずんだもん``, ``四国めたん``.
    style : str
        Style name, e.g. ``ノーマル``, ``あまあま``.
    directory : str
        The downloader's output directory.
    """

    def __init__(self, voice="ずんだもん", style="ノーマル", directory=VOICEVOX_DIR,
                 threads=0):
        import glob
        from voicevox_core.blocking import (Onnxruntime, OpenJtalk, Synthesizer,
                                            VoiceModelFile)
        libs = sorted(glob.glob(os.path.join(directory, "onnxruntime", "lib",
                                             "libvoicevox_onnxruntime.so*")))
        vvms = sorted(glob.glob(os.path.join(directory, "models", "vvms", "*.vvm")))
        dic = os.path.join(directory, "dict", "open_jtalk_dic_utf_8-1.11")
        if not libs or not vvms or not os.path.isdir(dic):
            raise FileNotFoundError(
                "VOICEVOX is not set up in {}: run `ros2 run jedy_speech setup_voicevox` "
                "(you will be asked to accept the voice terms)".format(directory))
        ort = Onnxruntime.load_once(filename=libs[-1])
        self._synth = Synthesizer(ort, OpenJtalk(dic), cpu_num_threads=threads)
        found = []
        for path in vvms:
            with VoiceModelFile.open(path) as model:
                for meta in model.metas:
                    for st in meta.styles:
                        found.append((meta.name, st.name))
                        if meta.name == voice and st.name == style:
                            self._synth.load_voice_model(model)
                            self._style = st.id
        if not hasattr(self, "_style"):
            raise ValueError("no voice {}/{} in {}; available: {}".format(
                voice, style, vvms, sorted(set(found))))
        self.credit = "VOICEVOX:{}".format(voice)
        self._synth.tts("あ", self._style)  # warm up

    def synthesize(self, text):
        """Return ``(int16 samples, sample_rate)``."""
        data = self._synth.tts(text, self._style)
        with wave.open(io.BytesIO(data)) as w:
            rate = w.getframerate()
            samples = np.frombuffer(w.readframes(w.getnframes()), np.int16)
        return samples, rate


def save_wav(path, samples, sample_rate):
    """Write int16 mono samples to a WAV file."""
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(sample_rate)
        w.writeframes(samples.astype(np.int16).tobytes())


def load_wav_16k(path):
    """A mono WAV file as float32 at 16 kHz (linear resampling)."""
    with wave.open(path) as w:
        sr, ch, width = w.getframerate(), w.getnchannels(), w.getsampwidth()
        raw = w.readframes(w.getnframes())
    if width != 2:
        raise ValueError("{}: only 16-bit WAV is supported".format(path))
    x = np.frombuffer(raw, np.int16).astype(np.float32) / 32768.0
    if ch > 1:
        x = x.reshape(-1, ch).mean(axis=1)
    if sr != SAMPLE_RATE:
        n = int(round(len(x) * SAMPLE_RATE / sr))
        x = np.interp(np.linspace(0, len(x), n, endpoint=False), np.arange(len(x)), x)
    return x.astype(np.float32)


# --------------------------------------------------------------- listening

#: Words the recogniser should expect (helps with "ジェディ" and the commands).
HOTWORDS = "ジェディ、右手、左手、両手、上げて、下げて、うなずいて、前に進んで、止まって"


class Recognizer:
    """Whisper "base" (int8, CPU) via faster-whisper, Japanese."""

    def __init__(self, threads=4):
        from faster_whisper import WhisperModel
        self.model = WhisperModel(model_path("faster-whisper-base"), device="cpu",
                                  compute_type="int8", cpu_threads=threads)

    def transcribe(self, audio):
        """Text of float32 16 kHz mono audio."""
        segments, _ = self.model.transcribe(
            audio, language="ja", beam_size=1, vad_filter=True,
            initial_prompt=HOTWORDS, condition_on_previous_text=False)
        return "".join(s.text for s in segments).strip()


def microphone_utterances(threshold=0.02, silence_s=0.8, max_s=10.0, min_s=0.3):
    """Yield one float32 16 kHz array per utterance from the microphone.

    A simple energy detector on 30 ms frames: an utterance starts when the
    level is above ``threshold`` (and above 3x the background), and ends
    after ``silence_s`` of quiet or ``max_s`` in total. ``arecord`` reads the
    default capture device (PulseAudio/PipeWire on a desktop).
    """
    if shutil.which("arecord") is None:
        raise RuntimeError("arecord not found (sudo apt install alsa-utils)")
    frame = int(0.03 * SAMPLE_RATE)
    proc = subprocess.Popen(["arecord", "-q", "-f", "S16_LE", "-r", str(SAMPLE_RATE),
                             "-c", "1", "-t", "raw"],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    noise, speech, quiet = 0.005, [], 0
    try:
        while True:
            raw = proc.stdout.read(frame * 2)
            if len(raw) < frame * 2:
                err = proc.stderr.read().decode(errors="replace").strip()
                raise RuntimeError("microphone stopped: {} (check `arecord -l`)".format(
                    err or "no data"))
            x = np.frombuffer(raw, np.int16).astype(np.float32) / 32768.0
            level = float(np.sqrt(np.mean(x * x)))
            loud = level > max(threshold, 3.0 * noise)
            if not speech:
                if loud:
                    speech = [x]
                    quiet = 0
                else:
                    noise = 0.95 * noise + 0.05 * level
                continue
            speech.append(x)
            quiet = 0 if loud else quiet + 1
            length = len(speech) * 0.03
            if quiet * 0.03 >= silence_s or length >= max_s:
                if length - quiet * 0.03 >= min_s:
                    yield np.concatenate(speech)
                speech, quiet = [], 0
    finally:
        proc.terminate()


def now_s():
    """Wall-clock seconds (for timing logs)."""
    return time.monotonic()
