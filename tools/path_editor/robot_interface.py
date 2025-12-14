#!/usr/bin/env python3
"""
Robot Mission Interface
Interface for the robot to retrieve and process mission data from the backend server
Uses unified ChipuRobot system for hardware control
"""

import requests
import json
import time
from typing import Dict, List, Optional, Tuple
import math

# Import unified robot system
try:
    from chipurobo_unified import ChipuRobot, GPIOPinManager
    ROBOT_SYSTEM_AVAILABLE = True
    print("✅ ChipuRobot unified system available")
except ImportError as e:
    print(f"⚠️ ChipuRobot system not available: {e}")
    ROBOT_SYSTEM_AVAILABLE = False

class RobotMissionInterface:
    def __init__(self, server_ip: str = "localhost", server_port: int = 5001):
        self.server_url = f"http://{server_ip}:{server_port}"
        self.current_mission = None
        self.robot_config = None
        
    def test_connection(self) -> bool:
        """Test connection to mission control server"""
        try:
            response = requests.get(f"{self.server_url}/status", timeout=5)
            if response.status_code == 200:
                status = response.json()
                print(f"✅ Connected to {status.get('server', 'Mission Control')}")
                print(f"   Version: {status.get('version', 'Unknown')}")
                print(f"   Missions available: {status.get('missions_stored', 0)}")
                return True
            else:
                print(f"❌ Server responded with status {response.status_code}")
                return False
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            return False
    
    def get_current_mission(self) -> Optional[Dict]:
        """Get the currently deployed mission"""
        try:
            response = requests.get(f"{self.server_url}/mission/current")
            if response.status_code == 200:
                data = response.json()
                if data.get('success'):
                    self.current_mission = data['mission']
                    print(f"📥 Retrieved mission: {self.current_mission.get('missionId', 'Unknown')}")
                    return self.current_mission
                else:
                    print("⚠️ No mission currently deployed")
                    return None
            else:
                print(f"❌ Failed to get mission: {response.status_code}")
                return None
        except Exception as e:
            print(f"❌ Error retrieving mission: {e}")
            return None
    
    def get_robot_config(self) -> Optional[Dict]:
        """Get robot configuration parameters"""
        try:
            response = requests.get(f"{self.server_url}/robot/config")
            if response.status_code == 200:
                data = response.json()
                if data.get('success'):
                    self.robot_config = data['config']
                    print(f"🤖 Robot config loaded:")
                    print(f"   Size: {self.robot_config.get('length', 'N/A')}ft x {self.robot_config.get('width', 'N/A')}ft")
                    print(f"   Max Speed: {self.robot_config.get('maxSpeed', 'N/A')} ft/s")
                    print(f"   Max Accel: {self.robot_config.get('maxAccel', 'N/A')} ft/s²")
                    return self.robot_config
                else:
                    print("⚠️ No robot configuration found")
                    return None
            else:
                print(f"❌ Failed to get robot config: {response.status_code}")
                return None
        except Exception as e:
            print(f"❌ Error retrieving robot config: {e}")
            return None
    
    def get_waypoints(self) -> List[Dict]:
        """Get waypoints from current mission"""
        if not self.current_mission:
            self.get_current_mission()
        
        if self.current_mission and 'path' in self.current_mission:
            waypoints = self.current_mission['path'].get('waypoints', [])
            print(f"🎯 Retrieved {len(waypoints)} waypoints")
            for i, wp in enumerate(waypoints):
                print(f"   WP{i+1}: ({wp.get('x', 'N/A'):.1f}ft, {wp.get('y', 'N/A'):.1f}ft)")
            return waypoints
        else:
            print("⚠️ No waypoints found in current mission")
            return []
    
    def get_field_elements(self) -> List[Dict]:
        """Get field elements (obstacles, zones, starting positions)"""
        if not self.current_mission:
            self.get_current_mission()
        
        if self.current_mission and 'field' in self.current_mission:
            elements = self.current_mission['field'].get('elements', [])
            print(f"🏟️ Retrieved {len(elements)} field elements")
            
            obstacles = [e for e in elements if e.get('type') == 'obstacle']
            zones = [e for e in elements if e.get('type') == 'zone']
            starts = [e for e in elements if e.get('type') == 'start']
            
            print(f"   🚧 Obstacles: {len(obstacles)}")
            print(f"   🎪 Zones: {len(zones)}")
            print(f"   🚀 Start positions: {len(starts)}")
            
            return elements
        else:
            print("⚠️ No field elements found in current mission")
            return []
    
    def convert_waypoint_to_robot_coords(self, waypoint: Dict, field_dims: Dict) -> Tuple[float, float]:
        """Convert field waypoint coordinates to robot coordinate system"""
        # Assuming robot coordinate system has (0,0) at bottom-left of field
        # and field coordinates are in feet
        x = waypoint.get('x', 0)
        y = waypoint.get('y', 0)
        return (x, y)
    
    def generate_trajectory(self, waypoints: List[Dict]) -> Optional[List[Dict]]:
        """Generate a trajectory from waypoints using robot constraints"""
        if not self.robot_config:
            self.get_robot_config()
        
        if not self.robot_config or not waypoints:
            return None
        
        trajectory_data = {
            'waypoints': waypoints,
            'robotConfig': self.robot_config
        }
        
        try:
            response = requests.post(f"{self.server_url}/path/generate", 
                                   json=trajectory_data,
                                   headers={'Content-Type': 'application/json'})
            
            if response.status_code == 200:
                data = response.json()
                if data.get('success'):
                    trajectory = data['trajectory']
                    print(f"📈 Generated trajectory with {len(trajectory)} points")
                    print(f"   Total time: {data.get('total_time', 'N/A'):.1f}s")
                    print(f"   Total distance: {data.get('total_distance', 'N/A'):.1f}ft")
                    return trajectory
                else:
                    print(f"❌ Trajectory generation failed: {data.get('error', 'Unknown error')}")
                    return None
            else:
                print(f"❌ Server error during trajectory generation: {response.status_code}")
                return None
        except Exception as e:
            print(f"❌ Error generating trajectory: {e}")
            return None
    
    def get_mission_summary(self) -> Dict:
        """Get a complete mission summary"""
        if not self.test_connection():
            return {'error': 'Cannot connect to mission control server'}
        
        mission = self.get_current_mission()
        config = self.get_robot_config()
        
        if not mission:
            return {'error': 'No mission currently deployed'}
        
        waypoints = self.get_waypoints()
        field_elements = self.get_field_elements()
        
        summary = {
            'mission_id': mission.get('missionId', 'Unknown'),
            'timestamp': mission.get('timestamp', 'Unknown'),
            'waypoints': len(waypoints),
            'field_elements': len(field_elements),
            'robot_config': config,
            'ready_for_execution': len(waypoints) > 0 and config is not None
        }
        
        print("\n🚀 MISSION SUMMARY:")
        print(f"   Mission ID: {summary['mission_id']}")
        print(f"   Waypoints: {summary['waypoints']}")
        print(f"   Field Elements: {summary['field_elements']}")
        print(f"   Ready for Execution: {'✅ YES' if summary['ready_for_execution'] else '❌ NO'}")
        
        return summary
    
    def execute_with_robot(self) -> bool:
        """Execute current mission using unified ChipuRobot system"""
        if not ROBOT_SYSTEM_AVAILABLE:
            print("❌ ChipuRobot system not available")
            return False
        
        # Get mission data
        summary = self.get_mission_summary()
        if not summary.get('ready_for_execution'):
            print("❌ Mission not ready for execution")
            return False
        
        try:
            # Initialize robot with config from server
            config = summary.get('robot_config', {})
            robot_config = {
                'wheelDiameter': config.get('wheelDiameter', 4.0),
                'wheelBase': config.get('wheelbase', 12.0),
                'encoderPPR': 11,  # Standard for DC motors with encoders
                'pwmFreq': config.get('pwmFreq', 1000)
            }
            
            robot = ChipuRobot(robot_config)
            
            # Execute the mission
            print("🚀 Starting mission execution...")
            robot.execute_mission(self.current_mission)
            
            return True
            
        except Exception as e:
            print(f"❌ Mission execution failed: {e}")
            return False
        finally:
            try:
                robot.cleanup()
            except:
                pass

def main():
    """Interactive robot mission interface"""
    print("🤖 ChipuRobot Mission Interface")
    print("=" * 40)
    
    # Create interface instance
    robot_interface = RobotMissionInterface()
    
    # Get mission summary
    summary = robot_interface.get_mission_summary()
    
    if summary.get('ready_for_execution'):
        print("\n🎯 Mission ready for execution!")
        
        if ROBOT_SYSTEM_AVAILABLE:
            print("\nOptions:")
            print("1. Execute mission on this robot")
            print("2. Just show mission data") 
            print("3. Exit")
            
            try:
                choice = input("\nEnter your choice (1-3): ").strip()
                
                if choice == '1':
                    print("\n🚀 Executing mission on robot hardware...")
                    success = robot_interface.execute_with_robot()
                    if success:
                        print("✅ Mission execution completed!")
                    else:
                        print("❌ Mission execution failed!")
                        
                elif choice == '2':
                    print("\n📊 Mission Data:")
                    waypoints = robot_interface.get_waypoints()
                    trajectory = robot_interface.generate_trajectory(waypoints)
                    if trajectory:
                        print(f"   {len(trajectory)} trajectory points generated")
                    
                elif choice == '3':
                    print("👋 Goodbye!")
                    
                else:
                    print("❌ Invalid choice")
                    
            except KeyboardInterrupt:
                print("\n👋 Interrupted by user")
        else:
            print("\n⚠️ Robot hardware system not available")
            print("   Can only display mission data")
            
            waypoints = robot_interface.get_waypoints()
            trajectory = robot_interface.generate_trajectory(waypoints)
            
    else:
        print("\n⚠️ Mission not ready for execution.")
        print("   Deploy a mission from the web interface first.")

if __name__ == '__main__':
    main()