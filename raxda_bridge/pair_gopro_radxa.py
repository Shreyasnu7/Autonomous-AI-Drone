import asyncio
import logging

# Ensure logging is set up to see the connection process
logging.basicConfig(level=logging.INFO)

try:
    from open_gopro import WirelessGoPro
except ImportError:
    print("Error: open-gopro is not installed on the Radxa. Please run: pip3 install open-gopro")
    exit(1)

async def pair_camera():
    print("\n📷 Searching for GoPro Hero 12 in Pairing Mode...")
    print("Scanning Bluetooth (this may take 5-15 seconds)...\n")
    try:
        # This automatically searches for the GoPro and attempts to pair
        gopro = WirelessGoPro()
        await gopro.open()
        
        print("\n✅ SUCCESS! Radxa is officially paired with your GoPro Hero 12!")
        print("We can now control the camera and view its livestream from the AI.")
        
        # We don't need to keep it open right now, just wanted to make the initial pairing save to the Radxa!
        await gopro.close()
        
    except Exception as e:
        print(f"\n❌ FAILED TO PAIR: {e}")
        print("Please make sure the GoPro screen still says 'Ready to connect'.")

if __name__ == "__main__":
    asyncio.run(pair_camera())
