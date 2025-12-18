#!/bin/bash
# ChipuRobo v0.5 - AI Camera (IMX500) Setup Script
# Run this script on your Raspberry Pi to install IMX500 firmware and dependencies

set -e

echo "🎯 ChipuRobot AI Camera Setup"
echo "=========================================="
echo "Setting up Raspberry Pi AI Camera (IMX500) support..."
echo ""

# Check if running on Raspberry Pi
if ! command -v raspi-config &> /dev/null; then
    echo "❌ Error: This script must be run on a Raspberry Pi"
    echo "   The AI Camera requires Raspberry Pi hardware"
    exit 1
fi

# Update system packages
echo "📦 Updating system packages..."
sudo apt update && sudo apt full-upgrade -y

# Install IMX500 firmware and models
echo "🤖 Installing IMX500 firmware and AI models..."
sudo apt install -y imx500-all

# Install development tools for custom models (optional)
echo "🔧 Installing AI Camera development tools..."
sudo apt install -y imx500-tools

# Install Python dependencies for AI Camera
echo "🐍 Installing Python dependencies for AI Camera..."
sudo apt install -y python3-opencv python3-munkres

# Install additional OpenCV dependencies
sudo apt install -y python3-cv-bridge

# Install pip dependencies from requirements.txt
echo "📋 Installing Python package requirements..."
pip3 install -r requirements.txt

# Verify camera connection
echo ""
echo "📷 Verifying AI Camera connection..."
if rpicam-hello --list-cameras | grep -q "imx500"; then
    echo "✅ AI Camera detected successfully!"
else
    echo "⚠️  Warning: AI Camera not detected"
    echo "   Please check:"
    echo "   1. Camera is properly connected to Raspberry Pi"
    echo "   2. Camera connector is secure"
    echo "   3. Camera is enabled in raspi-config"
    echo ""
    echo "   Run 'sudo raspi-config' → Interface Options → Camera → Enable"
fi

# Create configuration directories
echo "📁 Creating configuration directories..."
mkdir -p ~/.config/chipurobo
mkdir -p ~/.local/share/chipurobo/models

# Download example AI models (if available)
echo "📥 Setting up AI model examples..."
if [ -d "/usr/share/imx500-models" ]; then
    echo "✅ IMX500 models found in /usr/share/imx500-models/"
    ls -la /usr/share/imx500-models/ | head -5
else
    echo "⚠️  IMX500 models directory not found"
fi

# Set up camera configuration
echo "⚙️  Configuring camera settings..."
# Enable camera in config.txt if not already enabled
if ! grep -q "camera_auto_detect=1" /boot/config.txt; then
    echo "camera_auto_detect=1" | sudo tee -a /boot/config.txt
fi

echo ""
echo "🎉 AI Camera setup complete!"
echo ""
echo "Next steps:"
echo "1. Reboot your Raspberry Pi: sudo reboot"
echo "2. Test the setup: ./run.sh test"
echo "3. Run object detection demo: ./run.sh demo"
echo ""
echo "📖 For more information, see docs/HARDWARE_GUIDE.md"