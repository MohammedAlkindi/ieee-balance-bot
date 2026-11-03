import json

from bridge import EventGate, RobotState, build_prompt, parse_line


def test_parse_line_accepts_json_objects_only():
    assert parse_line('{"p": 1.5}\r\n') == {"p": 1.5}
    assert parse_line("ESP-ROM:esp32s3-20210327") is None
    assert parse_line('{"p": 1.5') is None
    assert parse_line("[1, 2]") is None


def test_summary_reports_latest_and_pitch_range():
    state = RobotState()
    for p in (0.5, -2.0, 8.5, 1.0):
        state.add({"p": p, "r": 3.0, "v": 0.1, "vbat": 11.7})
    s = state.summary()
    assert s["p"] == 1.0 and s["vbat"] == 11.7
    assert s["pitch_min"] == -2.0 and s["pitch_max"] == 8.5


def test_gate_enforces_cooldown_but_lets_priority_through():
    now = [100.0]
    gate = EventGate(cooldown_s=6.0, clock=lambda: now[0])
    assert gate.try_acquire()
    assert not gate.try_acquire(priority=True)  # busy speaking
    gate.release()
    now[0] += 2.0
    assert not gate.try_acquire()  # inside cooldown
    assert gate.try_acquire(priority=True)  # a fall or a judge's question still speaks


def test_prompt_is_compact_json_with_event_and_telemetry():
    state = RobotState()
    state.add({"p": 0.3})
    prompt = json.loads(build_prompt({"ev": "recovered", "peak": 9.6}, state))
    assert prompt["event"]["peak"] == 9.6
    assert prompt["telemetry"]["p"] == 0.3


def _sse(events):
    return "".join(f"event: {e['type']}\ndata: {json.dumps(e)}\n\n" for e in events).encode()


def test_speak_streams_claude_text_with_expected_request():
    import asyncio

    import anthropic
    import httpx2 as httpx

    from bridge import MODEL, speak

    seen = {}

    def handler(request: httpx.Request) -> httpx.Response:
        seen["body"] = json.loads(request.content)
        seen["beta"] = request.headers.get("anthropic-beta", "")
        msg = {"id": "msg_1", "type": "message", "role": "assistant", "model": MODEL, "content": [],
               "stop_reason": None, "stop_sequence": None, "usage": {"input_tokens": 50, "output_tokens": 1}}
        body = _sse([
            {"type": "message_start", "message": msg},
            {"type": "content_block_start", "index": 0, "content_block": {"type": "text", "text": ""}},
            {"type": "content_block_delta", "index": 0, "delta": {"type": "text_delta", "text": "I tipped ten degrees "}},
            {"type": "content_block_delta", "index": 0, "delta": {"type": "text_delta", "text": "and recovered."}},
            {"type": "content_block_stop", "index": 0},
            {"type": "message_delta", "delta": {"stop_reason": "end_turn", "stop_sequence": None}, "usage": {"output_tokens": 9}},
            {"type": "message_stop"},
        ])
        return httpx.Response(200, headers={"content-type": "text/event-stream"}, content=body)

    client = anthropic.AsyncAnthropic(api_key="test", http_client=httpx.AsyncClient(transport=httpx.MockTransport(handler)))
    text = asyncio.run(speak(client, '{"event":{"ev":"recovered"}}'))

    assert text == "I tipped ten degrees and recovered."
    assert seen["body"]["model"] == "claude-sonnet-5-5"
    assert seen["body"]["output_config"] == {"effort": "low"}
    assert seen["body"]["fallbacks"] == "default"
    assert seen["body"]["stream"] is True
    assert "server-side-fallback-2026-07-01" in seen["beta"]


def test_eleven_pcm_follows_stream_input_protocol(monkeypatch):
    """First message is a space, chunks pass through, then flush and an empty end-of-input message."""
    import asyncio
    import base64

    from bridge import eleven_pcm

    monkeypatch.setenv("ELEVENLABS_API_KEY", "test-key")
    sent, opened = [], {}

    class FakeSocket:
        def __init__(self):
            self.replies = asyncio.Queue()

        async def send(self, msg):
            sent.append(json.loads(msg))
            if sent[-1] == {"text": ""}:  # server answers only once input ends
                await self.replies.put(json.dumps({"audio": base64.b64encode(b"\x01\x00\x02\x00").decode()}))
                await self.replies.put(json.dumps({"audio": None, "isFinal": True}))

        def __aiter__(self):
            return self

        async def __anext__(self):
            return await self.replies.get()

    class FakeConnect:
        def __init__(self, url, additional_headers):
            opened["url"], opened["headers"] = url, additional_headers

        async def __aenter__(self):
            return FakeSocket()

        async def __aexit__(self, *exc):
            return False

    async def chunks():
        yield "Caught it "
        yield "in half a second."

    async def collect():
        return [pcm async for pcm in eleven_pcm(chunks(), connect=FakeConnect)]

    pcm = asyncio.run(collect())
    assert pcm == [b"\x01\x00\x02\x00"]
    assert "stream-input?model_id=eleven_flash_v2_5&output_format=pcm_16000" in opened["url"]
    assert opened["headers"] == {"xi-api-key": "test-key"}
    assert sent[0]["text"] == " " and "voice_settings" in sent[0]
    assert [m["text"] for m in sent[1:3]] == ["Caught it ", "in half a second."]
    assert sent[3] == {"text": " ", "flush": True} and sent[4] == {"text": ""}
