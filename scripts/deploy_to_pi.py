#!/usr/bin/env python3
"""
Deploy ChipuRobo to Raspberry Pi
Updated for new professional project structure
"""

import os
import sys
import subprocess
import json
from pathlib import Path


def run_command(cmd, description):
    """Run SSH command on remote Pi"""
    print(f"📡 {description}...")
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    
    if result.returncode != 0:
        print(f"❌ Failed: {result.stderr}")
        return False
    else:
        print(f"✅ Success")
        return True


def check_local_files():
    """Check if required files exist locally"""
    required_files = [
        "chipurobo/",
        "server/",
        "scripts/",
        "config/",
        "web/",
        "requirements.txt"
    ]
    
    missing = []
    for file_path in required_files:
        if not Path(file_path).exists():
            missing.append(file_path)
    
    if missing:
        print(f"❌ Missing required files/directories: {missing}")
        print("   Please run this from the project root directory")
        return False
        
    return True


def create_requirements_if_missing():
    """Create requirements.txt if it doesn't exist"""
    if not Path("requirements.txt").exists():
        requirements = """flask>=2.0.0
flask-cors>=3.0.0
toml>=0.10.0
RPi.GPIO>=0.7.0
adafruit-circuitpython-mpu6050
opencv-python
picamera2
numpy
"""
        with open("requirements.txt", "w") as f:
            f.write(requirements)
        print("📝 Created requirements.txt")


def deploy_to_pi(pi_ip, pi_user="pi", project_name="chipurobo"):
    """Deploy the professional ChipuRobo project to Raspberry Pi"""
    
    print(f"🚀 Deploying ChipuRobo to {pi_user}@{pi_ip}")
    print("=" * 50)
    
    # Check connection
    if not run_command(f'ssh {pi_user}@{pi_ip} "echo Connection OK"', "Testing SSH connection"):
        return False
    
    # Create project directory
    remote_dir = f"/home/{pi_user}/{project_name}"
    if not run_command(f'ssh {pi_user}@{pi_ip} "mkdir -p {remote_dir}"', f"Creating project directory {remote_dir}"):
        return False
    
    # Sync project files (excluding __pycache__ and .git)
    rsync_cmd = f'''rsync -avz --exclude="__pycache__" --exclude=".git" --exclude="*.pyc" \
        --exclude=".DS_Store" --exclude="tools/path_editor/" \
        ./ {pi_user}@{pi_ip}:{remote_dir}/'''
    
    if not run_command(rsync_cmd, "Syncing project files"):
        return False
    
    # Install Python dependencies
    install_cmd = f'''ssh {pi_user}@{pi_ip} "cd {remote_dir} && \
        python3 -m pip install --user -r requirements.txt"'''
    
    if not run_command(install_cmd, "Installing Python dependencies"):
        return False
    
    # Create systemd service for auto-start
    service_content = f'''[Unit]
Description=ChipuRobo Autonomous Robot
After=network.target

[Service]
Type=simple
User={pi_user}
WorkingDirectory=/home/{pi_user}/{project_name}
ExecStart=/usr/bin/python3 scripts/run_server.py --config config/production.toml
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target'''

    service_cmd = f'''ssh {pi_user}@{pi_ip} "echo '{service_content}' | \
        sudo tee /etc/systemd/system/chipurobo.service"'''
    
    if not run_command(service_cmd, "Creating systemd service"):
        return False
    
    # Enable and start service
    if not run_command(f'ssh {pi_user}@{pi_ip} "sudo systemctl enable chipurobo.service"', "Enabling ChipuRobo service"):
        return False
    
    # Create desktop shortcut for easy access
    desktop_content = f'''[Desktop Entry]
Version=1.0
Type=Application
Name=ChipuRobo Mission Control
Comment=Launch ChipuRobo web interface
Exec=chromium-browser --start-fullscreen http://localhost:5001
Icon=applications-games
Terminal=false
Categories=Game;Education;'''

    desktop_cmd = f'''ssh {pi_user}@{pi_ip} "mkdir -p /home/{pi_user}/Desktop && \
        echo '{desktop_content}' > /home/{pi_user}/Desktop/ChipuRobo.desktop && \
        chmod +x /home/{pi_user}/Desktop/ChipuRobo.desktop"'''
    
    run_command(desktop_cmd, "Creating desktop shortcut")
    
    # Test robot hardware
    test_cmd = f'ssh {pi_user}@{pi_ip} "cd {remote_dir} && python3 scripts/run_robot.py --test --duration 5"'
    if run_command(test_cmd, "Testing robot hardware"):
        print("🎯 Hardware test passed!")
    else:
        print("⚠️ Hardware test failed - check wiring")
    
    # Start the service
    if run_command(f'ssh {pi_user}@{pi_ip} "sudo systemctl start chipurobo.service"', "Starting ChipuRobo service"):
        print("\n🎉 Deployment Successful!")
        print(f"   🌐 Mission Control: http://{pi_ip}:5001")
        print(f"   🖥️ SSH Access: ssh {pi_user}@{pi_ip}")
        print(f"   📁 Project Directory: {remote_dir}")
        print("\n📋 Next Steps:")
        print("1. Open mission control web interface")
        print("2. Test robot movement and sensors")
        print("3. Create and execute autonomous missions")
        return True
    else:
        print("⚠️ Service start failed - check logs with:")
        print(f"   ssh {pi_user}@{pi_ip} 'sudo journalctl -u chipurobo.service -f'")
        return False


def main():
    """Main deployment function"""
    if len(sys.argv) < 2:
        print("Usage: python3 deploy_to_pi.py <pi_ip_address> [pi_username]")
        print("\nExamples:")
        print("   python3 scripts/deploy_to_pi.py raspberrypi.local")
        print("   python3 scripts/deploy_to_pi.py 192.168.1.100 pi")
        print("   python3 scripts/deploy_to_pi.py myrobot.local myuser")
        sys.exit(1)
    
    pi_ip = sys.argv[1] 
    pi_user = sys.argv[2] if len(sys.argv) > 2 else "pi"
    
    # Check local environment
    if not check_local_files():
        sys.exit(1)
    
    # Create requirements.txt if missing
    create_requirements_if_missing()
    
    # Deploy
    success = deploy_to_pi(pi_ip, pi_user)
    
    if success:
        print("\n🚀 ChipuRobo deployment complete!")
        print("   Your Raspberry Pi is ready for autonomous missions!")
    else:
        print("\n❌ Deployment failed")
        print("   Check error messages above")
        sys.exit(1)


if __name__ == "__main__":
    main()