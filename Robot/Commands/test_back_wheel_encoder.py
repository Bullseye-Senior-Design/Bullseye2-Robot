#!/usr/bin/env python3
"""
Test program for BackWheelEncoder subsystem.
This script tests the BackWheelEncoder class independently to verify encoder functionality.
"""

import sys
import time
import logging
from pathlib import Path

# Add the workspace root to `sys.path` so `from Robot...` imports work
# when this file is executed directly.
workspace_root = Path(__file__).resolve().parents[2]
if str(workspace_root) not in sys.path:
    sys.path.insert(0, str(workspace_root))

from Robot.subsystems.sensors.BackWheelEncoder import BackWheelEncoder

# Configure logging to see detailed output
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)

logger = logging.getLogger(__name__)


def test_back_wheel_encoder():
    """Test the BackWheelEncoder by reading and printing data."""
    
    logger.info("=" * 60)
    logger.info("Starting BackWheelEncoder Test")
    logger.info("=" * 60)
    
    try:
        # Create encoder instance
        encoder = BackWheelEncoder()
        logger.info("✓ BackWheelEncoder instance created")
        
        # Start the encoder
        encoder.start()
        logger.info("✓ BackWheelEncoder started")
        
        # Test for a duration
        test_duration = 30  # seconds
        print_interval = 2  # print every 2 seconds
        elapsed = 0
        
        logger.info(f"\nTesting encoder for {test_duration} seconds...")
        logger.info("(Rotate the back wheel to see encoder counts change)\n")
        
        start_time = time.time()
        
        while elapsed < test_duration:
            # Print current encoder data
            left_count = encoder.get_count_left()
            right_count = encoder.get_count_right()
            total_count = encoder.get_count_total_and_reset()
            velocity = encoder.get_velocity()
        
            elapsed = time.time() - start_time
            
            print(f"[{elapsed:6.1f}s] Left: {left_count:4d} | Right: {right_count:4d} | "
                  f"Total: {total_count:5d} | Velocity: {velocity:7.3f} m/s")
            
            time.sleep(print_interval)
        
        # Test reversing mode
        logger.info("\n" + "=" * 60)
        logger.info("Testing Reversing Mode (5 seconds)")
        logger.info("=" * 60)
        
        encoder.set_reversing(True)
        logger.info("Set reversing mode: ON")
        
        for i in range(5):
            velocity = encoder.get_velocity()
            logger.info(f"[Reversing] Velocity: {velocity:7.3f} m/s")
            time.sleep(1)
        
        encoder.set_reversing(False)
        logger.info("Set reversing mode: OFF")
        
        # Cleanup
        logger.info("\n" + "=" * 60)
        logger.info("Closing encoder...")
        encoder.close()
        logger.info("✓ BackWheelEncoder closed successfully")
        logger.info("=" * 60)
        
    except ImportError as e:
        logger.error(f"Import Error: {e}")
        logger.error("Make sure you're running this on a Raspberry Pi with GPIO support")
        logger.error("or in an environment where RPi.GPIO is available")
    except RuntimeError as e:
        logger.error(f"Runtime Error: {e}")
        logger.error("Make sure this script is run with appropriate GPIO permissions")
    except KeyboardInterrupt:
        logger.info("\nTest interrupted by user")
    except Exception as e:
        logger.error(f"Unexpected error: {e}", exc_info=True)


if __name__ == "__main__":
    test_back_wheel_encoder()
