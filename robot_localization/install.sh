#!/bin/bash

echo "=========================================="
echo "Installing Required Packages"
echo "=========================================="
echo ""
echo "Note: pybullet, numpy, and matplotlib are assumed to be already installed"
echo ""

# Install pybullet_tools if needed
echo "Installing pybullet-planning (pybullet_tools)..."
pip install pybullet-planning

echo ""
echo "=========================================="
echo "Installation Complete!"
echo "=========================================="
echo ""
echo "You can now run: python3 demo.py"
echo ""
