#!/bin/bash

# PMW3389 Navigation App Launcher
# This script activates the virtual environment and runs the navigation app

cd "$(dirname "$0")"

echo "🚗 Starting PMW3389 Dual Sensor Navigation App..."
echo "📡 Make sure your Teensy is connected and sensors are working!"
echo ""

# Check if virtual environment exists
if [ ! -d "venv" ]; then
    echo "❌ Virtual environment not found. Setting up..."
    python3 -m venv venv
    source venv/bin/activate
    pip install pygame pyserial
else
    echo "✅ Virtual environment found"
fi

# Run the navigation app
echo "🎮 Launching navigation app..."
"./venv/bin/python" navigation_app.py

echo ""
echo "👋 Navigation app closed. Thanks for using PMW3389 Navigation!"