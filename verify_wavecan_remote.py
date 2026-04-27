import json
import time
import urllib.request

BASE = "http://127.0.0.1:8080"


def get(path: str):
    with urllib.request.urlopen(BASE + path, timeout=3) as response:
        return json.loads(response.read().decode("utf-8"))


def post_cmd(payload: dict):
    req = urllib.request.Request(
        BASE + "/api/motor/cmd",
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
    )
    with urllib.request.urlopen(req, timeout=3) as response:
        return json.loads(response.read().decode("utf-8"))


for _ in range(60):
    try:
        print("HEALTH", get("/api/health"))
        break
    except Exception:
        time.sleep(0.25)
else:
    raise SystemExit("server not healthy")

status = get("/api/status")
print("STATUS_KEYS", sorted(status.keys()))
for motor in status.get("motors", []):
    print(
        "MOTOR",
        motor.get("motor_id"),
        "faults",
        motor.get("faults"),
        "can_debug",
        motor.get("can_debug"),
    )

print("ZERO_SUPPRESS_TEST_START")
print("SET_NONZERO", post_cmd({"id": 1, "cmd": "set", "value": 0.12}))
print("FORCE_ZERO", post_cmd({"id": 1, "cmd": "set", "value": 0.0, "force_stop": True}))
time.sleep(0.5)
status = get("/api/status")
for motor in status.get("motors", []):
    if motor.get("motor_id") == 1:
        print("M1_AFTER_FORCE_STOP", motor.get("output_percent"), motor.get("target_rpm"), motor.get("faults"))
