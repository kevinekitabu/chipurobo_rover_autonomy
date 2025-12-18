#!/bin/bash
# ChipuRobot v0.5 - Easy Run Script

echo "🤖 ChipuRobot v0.5 - Computer Vision Autonomous Rover"
echo "   Built for Kenya Science & Engineering Fair (KSEF) 2025"
echo ""

# Function to run with proper Python path
run_robot() {
    cd "$(dirname "$0")"
    PYTHONPATH=. python3 "$@"
}

# Check if an argument was provided
if [ $# -eq 0 ]; then
    echo "Usage: $0 [command]"
    echo ""
    echo "Available commands:"
    echo "  test          - Run system tests"
    echo "  test-ai       - Test AI Camera (IMX500) specifically"
    echo "  demo          - Quick demonstration"  
    echo "  person        - Enhanced person following demo"
    echo "  interactive   - Interactive control mode"
    echo "  ksef          - KSEF presentation demo"
    echo "  help          - Show this help"
    echo ""
    echo "Examples:"
    echo "  ./run.sh test"
    echo "  ./run.sh interactive"
    echo "  ./run.sh ksef"
    exit 1
fi

case "$1" in
    test)
        echo "🧪 Running system tests..."
        run_robot scripts/test_system.py
        ;;
    test-ai)
        echo "🤖 Testing AI Camera (IMX500)..."
        echo "   Checking camera detection, AI models, and inference..."
        run_robot scripts/test_ai_camera.py
        ;;
    demo)
        echo "🚀 Running quick demo..."
        run_robot examples/quick_demo.py
        ;;
    person)
        echo "👥 Running enhanced person following demo..."
        run_robot scripts/enhanced_person_demo.py
        ;;
    interactive)
        echo "🎮 Starting interactive mode..."
        run_robot scripts/main.py
        ;;
    ksef)
        echo "🏆 Starting KSEF presentation demo..."
        run_robot scripts/ksef_demo.py
        ;;
    help)
        echo "ChipuRobot v0.5 Help:"
        echo ""
        echo "This script makes it easy to run ChipuRobot programs with proper"
        echo "Python path configuration."
        echo ""
        echo "System Requirements:"
        echo "- Python 3.8+"
        echo "- Dependencies: pip install -r requirements.txt"
        echo "- Optional: Raspberry Pi 5 + AI Camera for hardware mode"
        echo ""
        echo "For more information, see README.md or docs/ directory"
        ;;
    *)
        echo "❌ Unknown command: $1"
        echo "Run '$0 help' for available commands"
        exit 1
        ;;
esac