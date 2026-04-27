"""
WaveCan Main Firmware Entry Point
Runs the motor controller, physics simulation, and web server
Works on both desktop (Python 3) and RP2350 (MicroPython)
"""

import asyncio
import sys
from platform import system as platform_system
from wavecan_platform import get_platform_info, get_can_bus_class, get_ticks_ms, log
from mock_sparkmax import MockMotorController, MockSPARKMAXConfig
from hardware_motor_controller import HardwareMotorController
from web_server import WebServer
from config import (
    CAN_BITRATE,
    CAN_INTERFACE,
    HTTP_HOST,
    HTTP_PORT,
    MOTOR_IDS,
    RUNTIME_MODE,
)

# Get the appropriate CAN bus for this platform
CANBusClass = get_can_bus_class()


class WaveCan:
    """Main application controller"""

    def __init__(self):
        self.platform_info = get_platform_info()
        self.runtime_mode = RUNTIME_MODE
        motor_ids = list(MOTOR_IDS)
        os_name = platform_system()
        log(f"[WaveCan] Operating System: {os_name}")
        log(f"[WaveCan] Starting on {self.platform_info['can_bus_class']} platform")
        log(f"[WaveCan] Runtime mode: {self.runtime_mode}")
        if self.runtime_mode == "socketcan":
            log(f"[WaveCan] ✓ Using REAL HARDWARE (SocketCAN)")
        else:
            log(f"[WaveCan] ✓ Using MOCK HARDWARE (simulation)")

        # Initialize CAN bus
        self.can_bus = CANBusClass(speed_kbps=int(CAN_BITRATE / 1000), name="WaveCanBus", channel=CAN_INTERFACE)

        if self.runtime_mode == "socketcan":
            discovered_ids = []

            # Active probe first so quiet but connected motors are still discovered.
            if hasattr(self.can_bus, "sweep_for_sparkmax_devices"):
                sweep = self.can_bus.sweep_for_sparkmax_devices(device_ids=range(1, 64), settle_ms=4)
                discovered_ids = list(sweep.get("found_ids", []))
                if discovered_ids:
                    log(f"[WaveCan] Active sweep discovered SPARK MAX IDs: {discovered_ids}")
                else:
                    log("[WaveCan] Active sweep found no SPARK MAX responses", "WARN")

            # Fallback to passive listen if active sweep found nothing.
            if not discovered_ids and hasattr(self.can_bus, "probe_bus_activity"):
                bus_activity = self.can_bus.probe_bus_activity(timeout_ms=1500)
                if bus_activity.get("traffic_detected"):
                    rev_devices = bus_activity.get("rev_devices", [])
                    if rev_devices:
                        discovered_ids = sorted({device["device_id"] for device in rev_devices})
                        log(f"[WaveCan] Passive probe detected active CAN devices: {discovered_ids}")
                    else:
                        log("[WaveCan] Detected CAN traffic on the bus")
                else:
                    log("[WaveCan] WARNING: No CAN traffic detected at startup; continuing in socketcan mode", "WARN")

            if discovered_ids:
                motor_ids = sorted({mid for mid in discovered_ids if 1 <= int(mid) <= 63})

        # Guard against invalid IDs from noisy discovery traffic.
        motor_ids = sorted({mid for mid in motor_ids if 1 <= int(mid) <= 63})

        if motor_ids != list(MOTOR_IDS):
            log(f"[WaveCan] Using discovered motor IDs: {motor_ids}")
        else:
            log(f"[WaveCan] Using configured motor IDs: {motor_ids}")

        # Initialize controller based on runtime mode
        if self.runtime_mode == "socketcan":
            self.motor_controller = HardwareMotorController(self.can_bus, motor_ids)
        else:
            motor_configs = [
                MockSPARKMAXConfig(mid, max_rpm=5700)
                for mid in motor_ids
            ]
            self.motor_controller = MockMotorController(self.can_bus, motor_configs)

        # Initialize web server
        self.web_server = WebServer(
            self.motor_controller,
            port=HTTP_PORT,
            host=HTTP_HOST,
            runtime_mode=self.runtime_mode
        )

        # Enable all motors
        self.motor_controller.enable_all()

        log("[WaveCan] Initialization complete")
        log(f"  Platform: {self.platform_info}")
        log(f"  HTTP: {HTTP_HOST}:{HTTP_PORT}")
        log(f"  Motors: {len(self.motor_controller.motors)}")
        log(f"  Effective runtime mode: {self.runtime_mode}")

    async def physics_loop(self):
        """
        Background task: Update motor physics at ~100Hz
        Runs periodically to advance motor state
        """
        update_interval_ms = 5  # Match the fast control cadence REV allows for native CAN send threads
        last_telemetry_ms = 0
        telemetry_interval_ms = 10  # Keep encoder telemetry fresher than the previous dashboard-visible lag

        while self.web_server.is_running:
            try:
                # Update physics for all motors
                self.motor_controller.update_physics(update_interval_ms)

                # Periodically broadcast telemetry
                current_time_ms = get_ticks_ms()
                if current_time_ms - last_telemetry_ms >= telemetry_interval_ms:
                    self.motor_controller.broadcast_telemetry()
                    last_telemetry_ms = current_time_ms

                # Sleep until next update
                await asyncio.sleep(update_interval_ms / 1000.0)

            except Exception as e:
                log(f"[Physics Loop] Error: {e}", "ERROR")
                await asyncio.sleep(0.01)

    async def run(self):
        """Start all systems: physics loop + web server"""
        try:
            # Start web server and physics loop concurrently
            await asyncio.gather(
                self.web_server.run(),
                self.physics_loop(),
                return_exceptions=True
            )
        except KeyboardInterrupt:
            log("[WaveCan] Shutdown requested")
        except Exception as e:
            log(f"[WaveCan] Fatal error: {e}", "ERROR")
        finally:
            self.shutdown()

    def shutdown(self):
        """Graceful shutdown"""
        log("[WaveCan] Shutting down...")
        self.web_server.stop()
        self.motor_controller.disable_all()
        log("[WaveCan] Shutdown complete")


async def main():
    """Main async entry point"""
    app = WaveCan()
    await app.run()


if __name__ == "__main__":
    log("=" * 60)
    log("WaveCan Motor Control Platform")
    log("=" * 60)

    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        log("\nShutdown by user")
        sys.exit(0)
    except Exception as e:
        log(f"Fatal error: {e}", "ERROR")
        sys.exit(1)
