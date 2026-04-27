import json
import sys
import time
from urllib import request, error


BASE = "http://127.0.0.1:8080"


def get_json(path):
    with request.urlopen(BASE + path, timeout=3) as resp:
        return json.loads(resp.read().decode("utf-8"))


def post_json(path, payload):
    data = json.dumps(payload).encode("utf-8")
    req = request.Request(
        BASE + path,
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with request.urlopen(req, timeout=3) as resp:
        return json.loads(resp.read().decode("utf-8"))


def summarize_motor(motor):
    faults = motor.get("faults", {}) or {}
    debug = motor.get("can_debug", {}) or {}
    return {
        "id": motor.get("motor_id"),
        "mode": motor.get("control_mode"),
        "rpm": motor.get("rpm"),
        "target_rpm": motor.get("target_rpm"),
        "output_percent": motor.get("output_percent"),
        "trusted": faults.get("trusted"),
        "active_faults": faults.get("active"),
        "sticky_faults": faults.get("sticky"),
        "api_class": debug.get("telemetry_api_class"),
        "rx_status0": debug.get("rx_status0_data_hex"),
        "rx_status1": debug.get("rx_status1_data_hex"),
        "rx_status2": debug.get("rx_status2_data_hex"),
    }


def wait_for_health(timeout_s=15):
    deadline = time.time() + timeout_s
    last_err = None
    while time.time() < deadline:
        try:
            health = get_json("/api/health")
            if health.get("status") == "ok":
                return health
        except Exception as exc:  # noqa: BLE001
            last_err = exc
        time.sleep(0.25)
    raise RuntimeError(f"server never became healthy: {last_err}")


def stop_all():
    try:
        status = get_json("/api/status")
        motors = status.get("motors", [])
    except Exception:
        motors = [{"motor_id": 1}, {"motor_id": 52}]
    ids = sorted({int(m.get("motor_id", 0)) for m in motors if m.get("motor_id")})
    for motor_id in ids:
        try:
            post_json("/api/motor/cmd", {"id": motor_id, "cmd": "stop_pid", "value": 0})
        except Exception:
            pass
        try:
            post_json("/api/motor/cmd", {"id": motor_id, "cmd": "set", "value": 0, "force_stop": True})
        except Exception:
            pass
    return ids


def main():
    motor_id = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    sequence = [250.0, 350.0, 450.0, 300.0]
    health = wait_for_health()
    print("health", json.dumps(health, sort_keys=True))

    try:
        print("initial_status", json.dumps(get_json("/api/status"), sort_keys=True))
        for target in sequence:
            result = post_json("/api/motor/cmd", {"id": motor_id, "cmd": "set_rpm", "value": target})
            print("set_rpm", target, json.dumps(result, sort_keys=True))
            for _ in range(8):
                time.sleep(0.35)
                status = get_json("/api/status")
                motors = status.get("motors", [])
                match = next((m for m in motors if int(m.get("motor_id", -1)) == motor_id), None)
                if match:
                    print("sample", json.dumps(summarize_motor(match), sort_keys=True))
        final_status = get_json("/api/status")
        print("final_status", json.dumps(final_status, sort_keys=True))
    finally:
        ids = stop_all()
        time.sleep(0.5)
        try:
            stopped = get_json("/api/status")
            print("stopped_status", json.dumps(stopped, sort_keys=True))
        except Exception as exc:  # noqa: BLE001
            print("stopped_status_error", repr(exc))
        print("stopped_ids", ids)


if __name__ == "__main__":
    main()
