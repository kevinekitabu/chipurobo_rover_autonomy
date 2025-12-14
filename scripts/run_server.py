#!/usr/bin/env python3
"""
ChipuRobot Server Launcher
Professional entry point for running the mission control server
"""

import sys
import argparse
from pathlib import Path

# Add project root to path
PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from chipurobo.utils.config_manager import ConfigManager, get_config
from chipurobo.utils.logger import setup_logging


def run_server(config_path: str = None, host: str = None, port: int = None, debug: bool = None):
    """Run the mission control server"""
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
    
    logger.info("🌐 Starting ChipuRobot Mission Control Server")
    
    # Get server configuration
    server_config = config.get_section('server')
    
    # Override with command line arguments
    final_host = host or server_config.get('host', '0.0.0.0')
    final_port = port or server_config.get('port', 5001)
    final_debug = debug if debug is not None else server_config.get('debug', False)
    
    logger.info(f"Server configuration:")
    logger.info(f"  Host: {final_host}")
    logger.info(f"  Port: {final_port}")
    logger.info(f"  Debug: {final_debug}")
    
    try:
        # Import and start Flask app
        from server.app import app, cleanup_robot
        import signal
        
        # Setup signal handlers
        def signal_handler(signum, frame):
            logger.info(f"Shutdown signal {signum} received")
            cleanup_robot()
            sys.exit(0)
        
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        # Start server
        app.run(host=final_host, port=final_port, debug=final_debug, threaded=True)
        
    except Exception as e:
        logger.error(f"Server startup failed: {e}")
        return False
    
    return True


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description='ChipuRobot Mission Control Server')
    parser.add_argument('--config', '-c', help='Configuration file path')
    parser.add_argument('--host', help='Server host address')
    parser.add_argument('--port', '-p', type=int, help='Server port number')
    parser.add_argument('--debug', '-d', action='store_true', help='Enable debug mode')
    parser.add_argument('--verbose', '-v', action='store_true', help='Verbose logging')
    
    args = parser.parse_args()
    
    # Adjust logging level
    if args.verbose:
        import logging
        logging.getLogger().setLevel(logging.DEBUG)
    
    success = run_server(
        config_path=args.config,
        host=args.host,
        port=args.port,
        debug=args.debug
    )
    
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()