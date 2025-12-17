#!/usr/bin/env python3
"""
Configuration Manager for ChipuRobo
Handles loading and managing robot configuration from TOML files
"""

import os
from pathlib import Path
from typing import Dict, Any, Optional

try:
    import toml
    TOML_AVAILABLE = True
except ImportError:
    TOML_AVAILABLE = False
    print("⚠️ toml library not available - install with: pip install toml")


class ConfigManager:
    """Configuration manager for ChipuRobot settings"""
    
    def __init__(self, config_path: Optional[str] = None):
        """
        Initialize configuration manager
        
        Args:
            config_path: Path to config file or directory
        """
        self.config_data = {}
        self.config_path = config_path
        
        if config_path:
            self.load_config(config_path)
        else:
            self.load_default_config()
    
    def load_config(self, config_path: str) -> bool:
        """Load configuration from file"""
        if not TOML_AVAILABLE:
            print("❌ Cannot load config - toml library not available")
            return False
        
        path = Path(config_path)
        
        # If it's a directory, look for config files
        if path.is_dir():
            # Try development first, then production
            dev_config = path / "development.toml"
            prod_config = path / "production.toml"
            
            if dev_config.exists():
                config_file = dev_config
                print("📄 Loading development configuration")
            elif prod_config.exists():
                config_file = prod_config
                print("📄 Loading production configuration")
            else:
                print("❌ No configuration files found in directory")
                return False
        else:
            config_file = path
        
        try:
            with open(config_file, 'r') as f:
                self.config_data = toml.load(f)
            
            print(f"✅ Configuration loaded from: {config_file}")
            return True
            
        except Exception as e:
            print(f"❌ Failed to load config: {e}")
            return False
    
    def load_default_config(self) -> None:
        """Load default configuration"""
        self.config_data = {
            'robot': {
                'wheel_diameter': 4.0,
                'wheel_base': 12.0,
                'robot_length': 12.0,
                'robot_width': 10.0,
                'max_speed': 3.0,
                'max_acceleration': 2.0,
                'pwm_frequency': 1000,
                'encoder_ppr': 11,
                'encoder_enabled': True,
                'imu_enabled': True,
                'vision_enabled': True,
                'camera_resolution': [640, 480]
            },
            'field': {
                'width': 16.5,
                'height': 8.2,
                'scale': 60
            },
            'server': {
                'host': '0.0.0.0',
                'port': 5001,
                'debug': False,
                'data_dir': 'data'
            },
            'logging': {
                'level': 'INFO',
                'file': 'logs/chipurobo.log',
                'max_size': '10MB',
                'backup_count': 5
            }
        }
        print("📄 Using default configuration")
    
    def get(self, section: str, key: str, default: Any = None) -> Any:
        """Get configuration value"""
        return self.config_data.get(section, {}).get(key, default)
    
    def get_section(self, section: str) -> Dict[str, Any]:
        """Get entire configuration section"""
        return self.config_data.get(section, {})
    
    def get_robot_config(self) -> Dict[str, Any]:
        """Get robot-specific configuration for ChipuRobot"""
        robot_config = self.get_section('robot')
        
        # Convert to ChipuRobot format
        return {
            'wheelDiameter': robot_config.get('wheel_diameter', 4.0),
            'wheelBase': robot_config.get('wheel_base', 12.0),
            'maxSpeed': robot_config.get('max_speed', 3.0),
            'maxAccel': robot_config.get('max_acceleration', 2.0),
            'pwmFreq': robot_config.get('pwm_frequency', 1000),
            'encoderPPR': robot_config.get('encoder_ppr', 11),
            'encoderEnabled': robot_config.get('encoder_enabled', True),
            'imuEnabled': robot_config.get('imu_enabled', True),
            'visionEnabled': robot_config.get('vision_enabled', True)
        }
    
    def set(self, section: str, key: str, value: Any) -> None:
        """Set configuration value"""
        if section not in self.config_data:
            self.config_data[section] = {}
        self.config_data[section][key] = value
    
    def save_config(self, output_path: str) -> bool:
        """Save current configuration to file"""
        if not TOML_AVAILABLE:
            print("❌ Cannot save config - toml library not available")
            return False
        
        try:
            with open(output_path, 'w') as f:
                toml.dump(self.config_data, f)
            
            print(f"✅ Configuration saved to: {output_path}")
            return True
            
        except Exception as e:
            print(f"❌ Failed to save config: {e}")
            return False
    
    def validate_config(self) -> Dict[str, Any]:
        """Validate configuration and return issues"""
        issues = []
        warnings = []
        
        # Check required sections
        required_sections = ['robot', 'field', 'server']
        for section in required_sections:
            if section not in self.config_data:
                issues.append(f"Missing required section: {section}")
        
        # Validate robot section
        robot = self.config_data.get('robot', {})
        if robot:
            # Check physical constraints
            if robot.get('wheel_diameter', 0) <= 0:
                issues.append("wheel_diameter must be positive")
            
            if robot.get('wheel_base', 0) <= 0:
                issues.append("wheel_base must be positive")
            
            if robot.get('max_speed', 0) <= 0:
                warnings.append("max_speed should be positive")
            
            # Check PWM frequency
            pwm_freq = robot.get('pwm_frequency', 1000)
            if pwm_freq < 100 or pwm_freq > 10000:
                warnings.append("pwm_frequency should be between 100-10000 Hz")
        
        # Validate server section
        server = self.config_data.get('server', {})
        if server:
            port = server.get('port', 5001)
            if not isinstance(port, int) or port < 1024 or port > 65535:
                issues.append("server port should be between 1024-65535")
        
        return {
            'valid': len(issues) == 0,
            'issues': issues,
            'warnings': warnings
        }
    
    def print_config(self) -> None:
        """Print current configuration"""
        print("📋 Current Configuration:")
        
        if TOML_AVAILABLE:
            print(toml.dumps(self.config_data))
        else:
            for section, values in self.config_data.items():
                print(f"\n[{section}]")
                for key, value in values.items():
                    print(f"{key} = {value}")


# Global config instance
_config_instance = None

def get_config() -> ConfigManager:
    """Get global configuration instance"""
    global _config_instance
    if _config_instance is None:
        # Try to load config from project root
        project_root = Path(__file__).parent.parent.parent
        config_dir = project_root / "config"
        
        if config_dir.exists():
            _config_instance = ConfigManager(str(config_dir))
        else:
            _config_instance = ConfigManager()  # Use defaults
    
    return _config_instance

def reload_config(config_path: Optional[str] = None) -> ConfigManager:
    """Reload configuration"""
    global _config_instance
    _config_instance = ConfigManager(config_path)
    return _config_instance


def load_config(config_name: str = "development") -> Dict[str, Any]:
    """
    Load configuration by name (standalone function)
    
    Args:
        config_name: Name of config file (without .toml extension)
        
    Returns:
        Configuration dictionary
    """
    project_root = Path(__file__).parent.parent.parent
    config_dir = project_root / "config"
    config_file = config_dir / f"{config_name}.toml"
    
    if not config_file.exists():
        print(f"⚠️ Config file not found: {config_file}")
        return {}
    
    if not TOML_AVAILABLE:
        print("⚠️ TOML library not available")
        return {}
    
    try:
        with open(config_file, 'r') as f:
            config_data = toml.load(f)
        print(f"📄 Loading {config_name} configuration")
        print(f"✅ Configuration loaded from: {config_file}")
        return config_data
    except Exception as e:
        print(f"❌ Failed to load config {config_file}: {e}")
        return {}