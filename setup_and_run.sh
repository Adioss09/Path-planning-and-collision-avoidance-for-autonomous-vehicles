#!/bin/bash

echo "=========================================="
echo " SIH26037 Environment Setup & Launcher    "
echo "=========================================="

# Check if python3 is installed
if ! command -v python3 &> /dev/null
then
    echo "Python3 could not be found. Please install Python3."
    exit
fi

# Create virtual environment if it doesn't exist
if [ ! -d "venv" ]; then
    echo "Creating Python virtual environment in ./venv..."
    python3 -m venv venv
else
    echo "Virtual environment already exists."
fi

# Activate environment
echo "Activating virtual environment..."
source venv/bin/activate

# Install requirements
echo "Installing dependencies..."
pip install -r requirements.txt

# Run the launcher
echo "Starting simulation launcher..."
python launcher.py
