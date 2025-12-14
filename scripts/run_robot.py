#!/usr/bin/env python3
"""
ChipuRobot Main Entry Point
Professional entry point for running the robot system
"""

import sys
import argparse
from pathlib import Path

# Add project root to path
PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from chipurobo.hardware.robot import ChipuRobot
from chipurobo.utils.config_manager import ConfigManager, get_config
from chipurobo.utils.logger import setup_logging, get_logger


def run_robot(config_path: str = None, test_mode: bool = False):
    """Run the robot system"""
    # Setup configuration
    if config_path:
        config = ConfigManager(config_path)
    else:
        config = get_config()
    
    # Setup logging
    log_config = config.get_section('logging')
    logger = setup_logging(
        log_file=log_config.get('file'),
        level=log_config.get('level', 'INFO')
    )
    
    logger.info("🚀 Starting ChipuRobot System")
    logger.info(f"Configuration: {config.config_path or 'default'}")
    
    # Validate configuration
    validation = config.validate_config()
    if not validation['valid']:
        logger.error("❌ Configuration validation failed:")
        for issue in validation['issues']:
            logger.error(f"  - {issue}")
        return False
    
    if validation['warnings']:
        logger.warning("⚠️ Configuration warnings:")
        for warning in validation['warnings']:
            logger.warning(f"  - {warning}")
    
    try:
        # Initialize robot
        robot_config = config.get_robot_config()
        robot = ChipuRobot(robot_config)
        
        logger.log_hardware_event("Robot", "Initialized", "All systems ready")
        
        if test_mode:
            logger.info("🧪 Running in test mode")
            run_test_sequence(robot, logger)
        else:
            logger.info("🎯 Robot ready for mission execution")
            # In production, robot waits for missions from server
            input("Press Enter to shutdown robot...")
        
        return True
        
    except Exception as e:
        logger.log_error_with_context(e, "robot initialization")
        return False
    
    finally:
        try:
            robot.cleanup()
            logger.info("✅ Robot shutdown complete")
        except:
            pass


def run_test_sequence(robot: ChipuRobot, logger):
    """Run basic test sequence"""
    logger.info("Starting test sequence...")
    
    # Test 1: System status
    status = robot.get_system_status()
    logger.log_robot_status(status['position_tracking'])
    
    # Test 2: Sensor calibration
    logger.info("Running sensor calibration...")
    calibration = robot.calibrate_sensors()
    logger.info(f"Calibration results: {calibration}")
    
    # Test 3: Basic movement (if not in simulation)
    import time
    
    logger.info("Testing basic movements...")
    robot.start_position_tracking()
    
    # Forward
    logger.info("Moving forward...")
    robot.drive_arcade(0.3, 0)
    time.sleep(2)
    robot.stop()
    
    # Turn
    logger.info("Turning...")
    robot.drive_arcade(0, 0.5)
    time.sleep(1)
    robot.stop()
    
    # Backward
    logger.info("Moving backward...")
    robot.drive_arcade(-0.3, 0)
    time.sleep(1)
    robot.stop()
    
    robot.stop_position_tracking()
    
    # Final status
    final_status = robot.get_sensor_data()
    logger.log_robot_status(final_status)
    logger.info("✅ Test sequence complete")


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description='ChipuRobot Control System')
    parser.add_argument('--config', '-c', help='Configuration file path')
    parser.add_argument('--test', '-t', action='store_true', help='Run in test mode')
    parser.add_argument('--verbose', '-v', action='store_true', help='Verbose logging')
    
    args = parser.parse_args()
    
    # Adjust logging level
    if args.verbose:
        import logging
        logging.getLogger().setLevel(logging.DEBUG)
    
    success = run_robot(config_path=args.config, test_mode=args.test)
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()