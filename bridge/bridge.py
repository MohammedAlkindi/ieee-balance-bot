"""Edge bridge: robot telemetry (serial JSON lines) -> Claude -> ElevenLabs -> speaker.

The ESP32-S3 prints one JSON object per line on its USB serial port:
  telemetry  {"t": 12345, "p": 1.2, "r": -3.4, "v": 0.05, "armed": 1, "vbat": 11.8, "loop_us": 2000, "jit_us": 14}
  events     {"ev": "recovered", "peak": 9.8, "ms": 640}  |  {"ev": "fell", "p": 41.0}  |  {"ev": "armed"}
             {"ev": "pad_connected"}  |  {"ev": "pad_lost"}  (gamepad; "vt"/"yaw" are its speed and turn targets)

The bridge keeps the last few seconds of telemetry, and on an event (or a
question typed into this terminal) asks Claude for one or two spoken sentences,
streams that text straight into ElevenLabs, and plays the returned PCM audio.

Instant reactions ("recovered", "fell") play a pre-rendered clip at once, because
a Claude + ElevenLabs round trip takes about 1.5 s; Claude's explanation follows.

Run without hardware or keys:   python bridge.py --fake --no-voice
Render the reaction clips once: python bridge.py --make-clips
Real robot:                     python bridge.py --port /dev/ttyACM0   (COM5 on Windows)
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import json
import math
import os
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

MODEL = "claude-sonnet-5-5"
SAMPLE_RATE = 16000  # matches ElevenLabs output_format=pcm_16000
ELEVEN_MODEL = "eleven_flash_v2_5"
ELEVEN_VOICE = os.environ.get("ELEVENLABS_VOICE_ID", "JBFqnCBsd6RMkjVDRZzb")
CLIP_DIR = Path(__file__).with_name("clips")
CLIPS = {  # event -> line rendered once with --make-clips, played instantly
    "recovered": "Whoa! Caught it.",
    "fell": "Oops. I fell over, so my motors are off.",
    "armed": "Okay, I'm balancing now.",
}

SYSTEM_PROMPT = """You are the voice of a small two-wheeled self-balancing robot at a student engineering showcase. Judges are listening.

Each message gives you one event and the last few seconds of the robot's own telemetry: pitch in degrees (positive = leaning forward), pitch rate in degrees per second, wheel speed in metres per second, battery volts, and control-loop timing.

Answer with one or two spoken sentences, at most 35 words. This text is read aloud by a speech engine that does not normalise numbers, so: no markdown, no lists, no emoji, no symbols, and write every number and unit as words ("about nine degrees", "six hundred milliseconds"). A short reaction sound has already played, so do not start with an exclamation.

What the robot is: an ESP32-S3 runs a 500 hertz loop. A complementary filter fuses the gyro and accelerometer into a pitch estimate. An inner loop turns pitch error and pitch rate into a wheel acceleration command for the stepper motors; an outer loop nudges the target pitch to bring wheel speed back to zero, so the robot stays in place.

Use the numbers you are given, never invent readings. If an event is a recovery, say how far it tipped and how fast it came back. If it fell, say so plainly and what tilt triggered the safety cutoff. If a judge asks a question, answer it from the telemetry and the description above."""


# ---------------------------------------------------------------- telemetry


def parse_line(line: str) -> dict | None:
    """Return the JSON object on a serial line, or None for noise and boot logs."""
    line = line.strip()
    if not line.startswith("{"):
        return None
    try:
        obj = json.loads(line)
    except json.JSONDecodeError:
        return None
    return obj if isinstance(obj, dict) else None


@dataclass
class RobotState:
    history: deque = field(default_factory=lambda: deque(maxlen=150))  # ~3 s at 50 Hz

    def add(self, sample: dict) -> None:
        self.history.append(sample)

    def summary(self) -> dict:
        """Compact numbers for the prompt: latest sample plus min/max pitch over the window."""
        if not self.history:
            return {}
        pitches = [s["p"] for s in self.history if isinstance(s.get("p"), (int, float))]
        latest = self.history[-1]
        out = {k: latest[k] for k in ("p", "r", "v", "vt", "yaw", "vbat", "loop_us", "jit_us", "armed") if k in latest}
        if pitches:
            out["pitch_min"] = round(min(pitches), 1)
            out["pitch_max"] = round(max(pitches), 1)
        out["window_s"] = round(len(self.history) / 50, 1)
        return out


class EventGate:
    """Speak at most once per `cooldown_s`, and never while already speaking."""

    def __init__(self, cooldown_s: float = 6.0, clock=time.monotonic):
        self.cooldown_s = cooldown_s
        self.clock = clock
        self.last = -math.inf
        self.busy = False

    def try_acquire(self, priority: bool = False) -> bool:
        if self.busy:
            return False
        if not priority and self.clock() - self.last < self.cooldown_s:
            return False
        self.busy = True
        return True

    def release(self) -> None:
        self.busy = False
        self.last = self.clock()


def build_prompt(trigger: dict, state: RobotState) -> str:
    return json.dumps({"event": trigger, "telemetry": state.summary()}, separators=(",", ":"))


# ------------------------------------------------------------------ sources


def serial_reader(port: str, baud: int, loop: asyncio.AbstractEventLoop, queue: asyncio.Queue) -> None:
    import serial  # pyserial

    with serial.Serial(port, baud, timeout=1) as ser:
        while True:
            raw = ser.readline()
            if raw:
                loop.call_soon_threadsafe(queue.put_nowait, raw.decode("utf-8", "replace"))


async def fake_robot(queue: asyncio.Queue) -> None:
    """Synthetic 50 Hz telemetry with a push every 12 s, for testing without hardware."""
    t0 = time.monotonic()
    next_push = 4.0
    while True:
        t = time.monotonic() - t0
        pitch = 0.4 * math.sin(t * 2.1)
        if next_push <= t < next_push + 0.8:
            k = (t - next_push) / 0.8
            pitch = 11.0 * math.sin(math.pi * k) * math.exp(-2.5 * k)
        elif t >= next_push + 0.8:
            await queue.put(json.dumps({"ev": "recovered", "peak": 9.6, "ms": 640}))
            next_push += 12.0
        await queue.put(json.dumps({"t": int(t * 1000), "p": round(pitch, 2), "r": round(20 * math.cos(t * 2.1), 1),
                                    "v": 0.02, "armed": 1, "vbat": 11.9, "loop_us": 2000, "jit_us": 12}))
        await asyncio.sleep(0.02)


def stdin_reader(loop: asyncio.AbstractEventLoop, queue: asyncio.Queue) -> None:
    for line in sys.stdin:
        if line.strip():
            loop.call_soon_threadsafe(queue.put_nowait, json.dumps({"ev": "question", "text": line.strip()}))


# ------------------------------------------------------------- claude + tts


async def claude_text(client, prompt: str):
    """Yield text chunks from Claude as they stream."""
    async with client.beta.messages.stream(
        model=MODEL,
        max_tokens=2000,
        system=SYSTEM_PROMPT,
        output_config={"effort": "low"},
        betas=["server-side-fallback-2026-07-01"],
        fallbacks="default",
        messages=[{"role": "user", "content": prompt}],
    ) as stream:
        async for text in stream.text_stream:
            yield text
        final = await stream.get_final_message()
    if final.stop_reason == "refusal":
        yield " I'll skip that one."


async def eleven_pcm(text_chunks, connect=None):
    """Stream text chunks into ElevenLabs stream-input and yield 16-bit mono PCM as it arrives."""
    if connect is None:
        import websockets

        connect = websockets.connect
    url = (f"wss://api.elevenlabs.io/v1/text-to-speech/{ELEVEN_VOICE}/stream-input"
           f"?model_id={ELEVEN_MODEL}&output_format=pcm_{SAMPLE_RATE}")
    async with connect(url, additional_headers={"xi-api-key": os.environ["ELEVENLABS_API_KEY"]}) as ws:
        await ws.send(json.dumps({"text": " ", "voice_settings": {"stability": 0.5, "similarity_boost": 0.8}}))

        async def pump():
            async for chunk in text_chunks:
                await ws.send(json.dumps({"text": chunk}))
            await ws.send(json.dumps({"text": " ", "flush": True}))  # speak whatever is buffered
            await ws.send(json.dumps({"text": ""}))  # end of input

        sender = asyncio.create_task(pump())
        try:
            async for message in ws:
                data = json.loads(message)
                if data.get("audio"):
                    yield base64.b64decode(data["audio"])
                if data.get("isFinal"):
                    break
            await sender
        finally:
            sender.cancel()


async def _once(text: str):
    yield text


class Speaker:
    """One open audio output; clips and streamed speech both write PCM to it."""

    def __init__(self):
        import sounddevice as sd

        self.out = sd.RawOutputStream(samplerate=SAMPLE_RATE, channels=1, dtype="int16")
        self.out.start()

    def play(self, pcm: bytes) -> None:
        self.out.write(pcm)


def play_clip(speaker, event: str) -> bool:
    path = CLIP_DIR / f"{event}.pcm"
    if speaker is None or not path.exists():
        return False
    speaker.play(path.read_bytes())
    return True


async def speak(client, prompt: str, speaker=None) -> str:
    """Stream Claude's reply into ElevenLabs and play it; with no speaker, print it."""
    spoken: list[str] = []

    async def tee():
        async for chunk in claude_text(client, prompt):
            spoken.append(chunk)
            if speaker is None:
                print(chunk, end="", flush=True)
            yield chunk

    if speaker is None:
        async for _ in tee():
            pass
        print()
    else:
        async for pcm in eleven_pcm(tee()):
            await asyncio.to_thread(speaker.play, pcm)  # blocking write, kept off the event loop
        print(f"[said] {''.join(spoken)}")
    return "".join(spoken)


async def make_clips() -> None:
    CLIP_DIR.mkdir(exist_ok=True)
    for event, line in CLIPS.items():
        pcm = b"".join([chunk async for chunk in eleven_pcm(_once(line))])
        (CLIP_DIR / f"{event}.pcm").write_bytes(pcm)
        print(f"[clips] {event}.pcm  {len(pcm) / (2 * SAMPLE_RATE):.1f} s  \"{line}\"")


# --------------------------------------------------------------------- main


async def run(args) -> None:
    import anthropic

    if args.make_clips:
        await make_clips()
        return
    if not args.no_voice and "ELEVENLABS_API_KEY" not in os.environ:
        sys.exit("ELEVENLABS_API_KEY is not set; set it, or run with --no-voice to print the text instead")

    client = anthropic.AsyncAnthropic()
    speaker = None if args.no_voice else Speaker()
    queue: asyncio.Queue = asyncio.Queue()
    loop = asyncio.get_running_loop()
    state = RobotState()
    gate = EventGate(cooldown_s=args.cooldown)

    if args.fake:
        asyncio.create_task(fake_robot(queue))
    else:
        threading.Thread(target=serial_reader, args=(args.port, args.baud, loop, queue), daemon=True).start()
    threading.Thread(target=stdin_reader, args=(loop, queue), daemon=True).start()

    async def handle(trigger: dict) -> None:
        try:
            await asyncio.to_thread(play_clip, speaker, trigger["ev"])  # instant reaction, if a clip exists
            await speak(client, build_prompt(trigger, state), speaker)
        except Exception as exc:  # keep the bridge alive through API and audio errors
            print(f"[bridge] speech failed: {exc!r}", file=sys.stderr)
        finally:
            gate.release()

    print("[bridge] running; type a question and press Enter to ask the robot")
    while True:
        obj = parse_line(await queue.get())
        if obj is None:
            continue
        if "ev" not in obj:
            state.add(obj)
            continue
        if gate.try_acquire(priority=obj["ev"] in ("question", "fell")):
            asyncio.create_task(handle(obj))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyACM0", help="ESP32-S3 USB serial port (COM5 on Windows)")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--fake", action="store_true", help="use synthetic telemetry instead of a serial port")
    ap.add_argument("--no-voice", action="store_true", help="print Claude's text instead of calling ElevenLabs")
    ap.add_argument("--cooldown", type=float, default=6.0, help="minimum seconds between spontaneous remarks")
    ap.add_argument("--make-clips", action="store_true", help="render the instant-reaction clips with ElevenLabs and exit")
    asyncio.run(run(ap.parse_args()))


if __name__ == "__main__":
    main()
