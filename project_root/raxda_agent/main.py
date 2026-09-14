# radxa_agent/main.py

AI_MODE_ENABLED = True  # later this comes from app checkbox

import asyncio
import requests

from ws_client import SafeWSClient
from safety_checks import TelemetryReader
from validator import validate_primitive
from plan_executor import simulate_execution
from telemetry_sim import create_telemetry_snapshot

from config import (
    PLAN_NEXT_ENDPOINT,
    AUTH_TOKEN,
    PLAN_POLL_INTERVAL,
    TELEMETRY_PUBLISH_INTERVAL,
    USE_MAVSDK,
    MAVSDK_CONNECTION_STR,
)

from radxa_exec.safety_gate import SafetyGate, SafetyLimits
from radxa_exec.executor import Executor
from radxa_exec.intent_receiver import IntentReceiver
from radxa_exec.actuators import Gimbal, Motion

from radxa_agent.heartbeat import Heartbeat
from radxa_agent.intent_forwarder import IntentForwarder
from radxa_agent.manual_override import ManualOverride
from radxa_agent.plan_to_intent import PlanToIntentAdapter
from radxa_agent.intent_arbiter import IntentArbiter
from radxa_agent.manual_input import ManualInput


# ---------------- PLAN POLLER ----------------

async def plan_poller(
    ws_client,
    telem_reader,
    plan_adapter,
    heartbeat,
    forwarder,
    arbiter,
    manual_input,
):
    while True:
        try:
            r = requests.get(
                PLAN_NEXT_ENDPOINT,
                headers={"Authorization": AUTH_TOKEN},
                timeout=10,
            )

            if r.status_code == 200:
                payload = r.json()
                plan = payload.get("plan")

                if plan:
                    print("📥 New plan received:", plan.get("action"))

                    safe_plan = validate_primitive(plan)
                    status = telem_reader.get_status()

                    if not status.get("ok_battery", True) or not status.get("ok_sats", True):
                        print("⚠️ Safety check failed. Rejecting plan.")
                        simulate_execution(
                            {
                                "action": "REJECTED",
                                "reason": "safety_failed",
                                "original": safe_plan,
                            },
                            status,
                        )
                    else:
                        # ---- AI intent (only if enabled) ----
                        ai_intent = None
                        if AI_MODE_ENABLED:
                            ai_intent = plan_adapter.convert(safe_plan)

                        # ---- Manual intent (joystick/app) ----
                        manual_intent = manual_input.get()  # None if no input

                        # ---- Final intent ----
                        final_intent = arbiter.blend(ai_intent, manual_intent)

                        if final_intent is not None:
                            heartbeat.tick()
                            forwarder.forward(final_intent)

                        await ws_client.send(
                            {
                                "target": "server",
                                "type": "plan_executed",
                                "drone_id": "drone_1",
                                "plan": safe_plan,
                            }
                        )

        except Exception as e:
            print("⚠️ Plan poll error:", e)

        await asyncio.sleep(PLAN_POLL_INTERVAL)


# ---------------- TELEMETRY ----------------

async def telemetry_publisher(ws_client, telem_reader):
    while True:
        try:
            status = telem_reader.get_status()
            snapshot = create_telemetry_snapshot()

            snapshot.update(
                {
                    "battery": status.get("battery"),
                    "sats": status.get("sats"),
                    "ok_battery": status.get("ok_battery"),
                    "ok_sats": status.get("ok_sats"),
                }
            )

            await ws_client.send(
                {
                    "target": "server",
                    "type": "telemetry",
                    "drone_id": "drone_1",
                    "data": snapshot,
                }
            )

        except Exception as e:
            print("⚠️ Telemetry publish failed", e)

        await asyncio.sleep(TELEMETRY_PUBLISH_INTERVAL)


# ---------------- MAIN ----------------

async def main():
    ws = SafeWSClient("radxa")
    await ws.connect()

    telem = TelemetryReader(
        use_mavsdk=USE_MAVSDK,
        connection_str=MAVSDK_CONNECTION_STR,
    )
    await telem.start()

    # ---- EXECUTION LAYER ----
    safety = SafetyGate(SafetyLimits())
    gimbal = Gimbal()
    motion = Motion()

    executor = Executor(safety, gimbal, motion)
    intent_receiver = IntentReceiver(executor)

    heartbeat = Heartbeat(safety)
    manual_override = ManualOverride(safety)
    forwarder = IntentForwarder(intent_receiver, manual_override)

    plan_adapter = PlanToIntentAdapter()
    arbiter = IntentArbiter()
    manual_input = ManualInput()

    await asyncio.gather(
        plan_poller(
            ws,
            telem,
            plan_adapter,
            heartbeat,
            forwarder,
            arbiter,
            manual_input,
        ),
        telemetry_publisher(ws, telem),
    )


if __name__ == "__main__":
    asyncio.run(main())