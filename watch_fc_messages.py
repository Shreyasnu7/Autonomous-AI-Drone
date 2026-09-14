"""
Standalone laptop listener — no AI stack needed. Connects to the Radxa bridge over Tailscale and
prints/beeps every FC status message (AutoTune Success/Failed, arming errors, etc.) live while you
fly on RC. Run this BEFORE takeoff, leave it running in a terminal on your desk.

Usage: python watch_fc_messages.py [radxa_tailscale_ip]
Default IP is the one in laptop_ai/config.py (CUBIE_TAILSCALE_IP).
"""
import asyncio, json, subprocess, sys, time

try:
    import winsound
    def beep(freq=1500, ms=350): winsound.Beep(freq, ms)
except ImportError:
    def beep(freq=1500, ms=350): print("\a", end="", flush=True)

import websockets

DEFAULT_IP = "100.89.83.125"  # Radxa Cubie A7Z Tailscale IP (update if it changes)

def toast(title, message):
    """Native Windows 10/11 toast notification via PowerShell (no extra pip package needed)."""
    ps = f'''
[Windows.UI.Notifications.ToastNotificationManager, Windows.UI.Notifications, ContentType = WindowsRuntime] | Out-Null
[Windows.Data.Xml.Dom.XmlDocument, Windows.Data.Xml.Dom.XmlDocument, ContentType = WindowsRuntime] | Out-Null
$template = @"
<toast><visual><binding template="ToastGeneric"><text>{title}</text><text>{message}</text></binding></visual></toast>
"@
$xml = New-Object Windows.Data.Xml.Dom.XmlDocument
$xml.LoadXml($template)
$toast = New-Object Windows.UI.Notifications.ToastNotification $xml
[Windows.UI.Notifications.ToastNotificationManager]::CreateToastNotifier("Drone Watcher").Show($toast)
'''
    try:
        subprocess.Popen(["powershell", "-NoProfile", "-Command", ps],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
        pass

async def watch(ip):
    url = f"ws://{ip}:8000"
    print(f"Connecting to bridge at {url} ...")
    while True:
        try:
            async with websockets.connect(url, ping_interval=20, ping_timeout=10) as ws:
                print("✅ Connected. Watching for FC messages (AutoTune, arming, etc.)...\n")
                async for raw in ws:
                    try:
                        data = json.loads(raw)
                    except Exception:
                        continue
                    if data.get("type") != "fc_status_text":
                        continue
                    p = data.get("payload", {})
                    text = p.get("text", "")
                    sev = p.get("severity", 6)
                    tstr = time.strftime("%H:%M:%S")
                    tag = "🔴 CRITICAL" if sev <= 3 else "🟡 WARNING" if sev <= 5 else "🟢 INFO"
                    print(f"[{tstr}] {tag}: {text}")
                    low = text.lower()
                    if "autotune" in low and "success" in low:
                        print("\n" + "=" * 50)
                        print("  🎉 AUTOTUNE SUCCESS — you can now land + disarm to save,")
                        print("     or double-toggle the switch to save while airborne.")
                        print("=" * 50 + "\n")
                        beep(2000, 500); beep(2500, 500)
                        toast("🎉 AutoTune Success", "Land + disarm to save, or double-toggle SwB in the air.")
                    elif "autotune" in low and "fail" in low:
                        print("\n⚠️ AUTOTUNE FAILED — it will revert to previous gains.\n")
                        beep(600, 700)
                        toast("⚠️ AutoTune Failed", "Reverted to previous gains — try again.")
                    elif sev <= 3:
                        toast("🔴 FC Critical Message", text)
        except Exception as e:
            print(f"Disconnected ({e}); retrying in 3s...")
            await asyncio.sleep(3)

if __name__ == "__main__":
    ip = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_IP
    try:
        asyncio.run(watch(ip))
    except KeyboardInterrupt:
        print("\nStopped.")
