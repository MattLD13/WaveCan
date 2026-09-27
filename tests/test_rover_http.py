import asyncio
import json

from web_server import HTTPRequest, WebServer


class LiveOutputRecorder:
    """HTTP API test spy without a CAN adapter or motor simulation."""

    def __init__(self):
        self.motors = {1: object(), 2: object()}
        self.outputs = []
        self.enabled = False
        self.disabled = 0

    def enable_all(self):
        self.enabled = True

    def disable_all(self):
        self.enabled = False
        self.disabled += 1

    def set_motor_output(self, motor_id, value):
        self.outputs.append((motor_id, value))


def request(body):
    return HTTPRequest(f"POST /api/rover/drive HTTP/1.1\r\nContent-Length: {len(body)}\r\n\r\n{body}")


def response_json(response):
    return json.loads(response.body)


def test_rover_http_arm_drive_and_stop_route():
    controller = LiveOutputRecorder()
    server = WebServer(controller)

    armed = asyncio.run(server.handle_rover_drive_command(request('{"op":"arm"}')))
    assert armed.status == 200 and controller.enabled

    driven = asyncio.run(server.handle_rover_drive_command(request(
        '{"op":"drive","left":0.5,"right":-0.25,"left_ids":[1],"right_ids":[2]}'
    )))
    assert driven.status == 200
    assert controller.outputs == [(1, 0.5), (2, -0.25)]

    stopped = asyncio.run(server.handle_rover_drive_command(request('{"op":"stop"}')))
    assert response_json(stopped)["stopped"] is True
    assert not controller.enabled
    assert controller.disabled == 1


def test_rover_http_rejects_disarmed_or_overlapping_drive():
    controller = LiveOutputRecorder()
    server = WebServer(controller)
    disarmed = asyncio.run(server.handle_rover_drive_command(request(
        '{"op":"drive","left":0.4,"right":0.4,"left_ids":[1],"right_ids":[2]}'
    )))
    assert disarmed.status == 409

    asyncio.run(server.handle_rover_drive_command(request('{"op":"arm"}')))
    overlap = asyncio.run(server.handle_rover_drive_command(request(
        '{"op":"drive","left":0.4,"right":0.4,"left_ids":[1],"right_ids":[1]}'
    )))
    assert overlap.status == 400
    assert controller.outputs == []
