"""Live WaveCan entry point for Linux SocketCAN motor control."""

import asyncio
import logging
import os
import sys
from platform import system as platform_system

from config import CAN_BITRATE, CAN_INTERFACE, HTTP_HOST, HTTP_PORT, MOTOR_IDS
from rover_control.runtime import RoverSafetyGate
from sparkmax_can import SparkMaxController
from wavecan_platform import get_platform_info, log
from web_server import WebServer

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")


class WaveCan:
    """Live motor-control application."""

    def __init__(self):
        if not sys.platform.startswith("linux"):
            raise RuntimeError("WaveCan live motor control requires Linux with SocketCAN")

        self.platform_info = get_platform_info()
        log(f"[WaveCan] Operating System: {platform_system()}")
        log("[WaveCan] Starting live SPARK MAX control over SocketCAN")

        self.motor_controller = SparkMaxController(
            channel=CAN_INTERFACE,
            bitrate=CAN_BITRATE,
            motor_ids=MOTOR_IDS,
            max_rpm=5700.0,
            discover=True,
            auto_start=True,
        )
        self.rover_safety_gate = RoverSafetyGate()
        try:
            self.web_server = WebServer(
                self.motor_controller,
                port=HTTP_PORT,
                host=HTTP_HOST,
                rover_safety_gate=self.rover_safety_gate,
            )
        except Exception:
            self.motor_controller.close()
            raise

        self.can_bus = self.motor_controller.can_bus
        self.ble_server = None
        if os.getenv("WAVECAN_BLE_ENABLED", "1").strip().lower() not in {"0", "false", "no", "off"}:
            try:
                from rover_control.pi_ble import PiBleServer

                self.ble_server = PiBleServer(
                    self.motor_controller,
                    name=os.getenv("WAVECAN_BLE_NAME", "WaveCan Rover"),
                    watchdog_ms=int(os.getenv("WAVECAN_BLE_WATCHDOG_MS", "600")),
                    safety_gate=self.rover_safety_gate,
                )
                self.ble_server.start()
                log("[WaveCan] Bluetooth LE rover control is advertising")
            except Exception as exc:
                self.ble_server = None
                log(f"[WaveCan] Bluetooth LE rover control unavailable: {exc}", "WARN")
        log("[WaveCan] Live hardware initialization complete")
        log(f"  Platform: {self.platform_info}")
        log(f"  HTTP: {HTTP_HOST}:{HTTP_PORT}")
        log(f"  Motors: {len(self.motor_controller.motors)}")

    async def run(self):
        """Run the web API while SparkMaxController services SocketCAN."""
        try:
            await self.web_server.run()
        except KeyboardInterrupt:
            log("[WaveCan] Shutdown requested")
        except Exception as exc:
            log(f"[WaveCan] Fatal error: {exc}", "ERROR")
        finally:
            self.shutdown()

    def shutdown(self):
        """Stop the motors through the high-level controller and close CAN."""
        log("[WaveCan] Shutting down live motors...")
        if self.ble_server is not None:
            self.ble_server.stop()
        self.web_server.stop()
        self.motor_controller.close()
        log("[WaveCan] Shutdown complete")


async def main():
    app = WaveCan()
    await app.run()


if __name__ == "__main__":
    log("WaveCan live SPARK MAX control")
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        log("Shutdown by user")
        sys.exit(0)
    except Exception as exc:
        log(f"Fatal error: {exc}", "ERROR")
        sys.exit(1)
