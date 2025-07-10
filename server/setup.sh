#!/bin/bash

# HERD Wildlife Detection System - Server Setup Script
# This script sets up the FastAPI backend server

echo "🌿 HERD Wildlife Detection System - Server Setup"
echo "================================================="

# Check if Python 3.8+ is installed
if ! command -v python3 &> /dev/null; then
    echo "❌ Python 3 is not installed. Please install Python 3.8 or higher."
    exit 1
fi

python_version=$(python3 -c 'import sys; print(".".join(map(str, sys.version_info[:2])))')
echo "✅ Python version: $python_version"

# Create virtual environment
echo "📦 Creating virtual environment..."
python3 -m venv venv

# Activate virtual environment
echo "🔧 Activating virtual environment..."
source venv/bin/activate

# Upgrade pip
echo "⬆️ Upgrading pip..."
pip install --upgrade pip

# Install dependencies
echo "📚 Installing dependencies..."
pip install -r requirements.txt

# Create necessary directories
echo "📁 Creating directories..."
mkdir -p stored_images
mkdir -p logs

# Create environment configuration file
echo "⚙️ Creating environment configuration..."
cat > .env << EOF
# HERD Backend Configuration
BACKEND_URL=http://localhost:8000
CDN_BASE_URL=http://localhost:8000/images
MAX_DETECTIONS_HISTORY=1000
IMAGES_DIR=stored_images
LOG_LEVEL=INFO

# Database Configuration (Optional)
# DATABASE_URL=postgresql://user:password@localhost/herd_db
# MONGO_URL=mongodb://localhost:27017/herd

# Firebase Configuration (Optional)
# FIREBASE_PROJECT_ID=your-project-id
# FIREBASE_PRIVATE_KEY_PATH=firebase-private-key.json

# Security
JWT_SECRET_KEY=your-secret-key-change-in-production
API_KEY=your-api-key-for-gateways
EOF

# Create systemd service file (optional)
echo "🔧 Creating systemd service file..."
cat > herd-backend.service << EOF
[Unit]
Description=HERD Wildlife Detection Backend
After=network.target

[Service]
Type=simple
User=$USER
WorkingDirectory=$(pwd)
Environment=PATH=$(pwd)/venv/bin
ExecStart=$(pwd)/venv/bin/uvicorn main:app --host 0.0.0.0 --port 8000
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

echo "📝 Service file created: herd-backend.service"
echo "   To install: sudo cp herd-backend.service /etc/systemd/system/"
echo "   To enable: sudo systemctl enable herd-backend"
echo "   To start: sudo systemctl start herd-backend"

# Test the installation
echo "🧪 Testing installation..."
echo "Starting development server..."
echo "Access the API at: http://localhost:8000"
echo "API documentation at: http://localhost:8000/docs"
echo ""
echo "Press Ctrl+C to stop the server"
echo ""

# Start the development server
uvicorn main:app --host 0.0.0.0 --port 8000 --reload
