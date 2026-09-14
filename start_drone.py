import sys
import os
import asyncio

# Setup paths so all ai modules can find each other
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.append(project_root)
sys.path.append(os.path.join(project_root, 'ai', 'camera_brain'))
sys.path.append(os.path.join(project_root, 'ai', 'camera_brain', 'laptop_ai'))

from dotenv import load_dotenv

# Load environment variables from .env file FIRST before any AI models initialize
load_dotenv(os.path.join(project_root, '.env'))
# Fallback: if the user put their key in the template file, load that too
load_dotenv(os.path.join(project_root, '.env.template'))

from ai.camera_brain.laptop_ai.director_core import DirectorCore

async def main():
    print("==================================================")
    print("🚁 CONTINUOUS TWO-BRAIN DRONE ARCHITECTURE INITIATED")
    print("==================================================")
    
    # Initialize the core
    director = DirectorCore()
    
    # Start all concurrent loops (Vision, Local ER, Continuous Gemini)
    await director.start()

    # Keep the main thread alive while async loops run
    while True:
        await asyncio.sleep(1)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nShutdown signal received. Stopping Core...")
