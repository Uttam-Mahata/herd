# HERD Wildlife Detection System - Implementation Guide

## Overview
This implementation provides the complete firmware and server code for the HERD jungle wildlife intrusion detection system using ESP-NOW mesh networking.

## Architecture Components

### 1. Node Firmware (`/firmware/node/node_main.ino`)
**ESP32-CAM sensor nodes with:**
- PIR motion detection
- Camera image capture
- ESP-NOW mesh networking
- Deep sleep power management
- Multi-gateway failover
- Store-and-forward capability

**Key Features:**
- Battery life: 2-6 months with deep sleep
- Automatic image transmission via mesh
- Smart gateway discovery and routing
- Low power consumption (1-3mA average)

### 2. Gateway Firmware (`/firmware/gateway/gateway_main.ino`) 
**ESP32 gateway nodes with:**
- ESP-NOW mesh receiver
- Image packet reassembly
- WiFi hotspot for Android communication
- HTTP server for TFLite app interface
- Multi-gateway coordination

**Key Features:**
- Automatic image reconstruction from packets
- Real-time communication with Android app
- Store images locally on SPIFFS
- Gateway beacon broadcasting

### 3. Android TFLite App (`/app/android_gateway/TFLiteGatewayService.java`)
**Android service for AI processing:**
- TensorFlow Lite wildlife detection model
- HTTP communication with ESP32 gateway
- Real-time image processing
- Backend API integration

**Key Features:**
- Local TFLite inference for speed
- Wildlife vs false positive filtering
- Automatic backend notification
- Offline processing capability

### 4. FastAPI Backend (`/server/main.py`)
**Python backend server with:**
- REST API for gateways and mobile apps
- Wildlife detection storage
- Push notification service
- System monitoring and statistics

**Key Features:**
- Multi-gateway management
- Duplicate detection filtering
- Real-time alerts and notifications
- Comprehensive API endpoints

## Hardware Requirements

### Sensor Node (Per Unit)
- ESP32-CAM module
- PIR motion sensor (HC-SR501)
- MicroSD card (8GB+)
- 3000-5000mAh LiPo battery
- Solar panel (5-10W)
- TP4056 charging module
- Weatherproof enclosure

### Gateway Node (Per Zone)
- ESP32 development board
- Android smartphone/tablet
- 4G/LTE connectivity
- Large battery bank (10000mAh+)
- Solar panel (20W+)
- Weatherproof enclosure

## Installation & Setup

### 1. Flash Node Firmware
```bash
# Configure each node with unique ID
#define NODE_ID "HERD_NODE_001"  // Change for each node

# Upload via Arduino IDE or PlatformIO
arduino-cli compile --fqbn esp32:esp32:esp32cam firmware/node/node_main.ino
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32cam firmware/node/
```

### 2. Flash Gateway Firmware
```bash
# Configure gateway zone and priority
#define GATEWAY_ID "HERD_GW_001"
#define GATEWAY_ZONE "North"
#define GATEWAY_PRIORITY 1

# Upload via Arduino IDE
arduino-cli compile --fqbn esp32:esp32:esp32 firmware/gateway/gateway_main.ino
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 firmware/gateway/
```

### 3. Setup Backend Server
```bash
cd server/
pip install -r requirements.txt
uvicorn main:app --host 0.0.0.0 --port 8000 --reload
```

### 4. Install Android App
```bash
# Build and install TFLite gateway app
# Add wildlife detection model to assets/wildlife_detection.tflite
# Configure BACKEND_URL in TFLiteGatewayService.java
```

## Configuration

### Node Configuration (`node_main.ino`)
```cpp
#define NODE_ID "HERD_NODE_XXX"           // Unique per node
#define PIR_PIN GPIO_NUM_13               // PIR sensor pin
#define DEEP_SLEEP_DURATION 300000000     // 5 minutes
#define LOW_BATTERY_THRESHOLD 3.3         // Battery cutoff voltage
```

### Gateway Configuration (`gateway_main.ino`)
```cpp
#define GATEWAY_ID "HERD_GW_XXX"          // Unique per gateway
#define GATEWAY_ZONE "North/South/East"   // Zone identifier
#define HOTSPOT_SSID "HERD_Gateway_XXX"   // WiFi hotspot name
#define HOTSPOT_PASSWORD "HerdGateway2024" // WiFi password
```

### Backend Configuration (`main.py`)
```python
BACKEND_URL = "http://your-server.com/api"
CDN_BASE_URL = "http://your-cdn.com/images"
MAX_DETECTIONS_HISTORY = 1000
```

## Deployment Strategy

### Network Topology
```
                    [Gateway]
                   (Hill/Tower)
                   📡 ESP32+ESP-NOW
                   📱 Android+4G
                        |
        ┌───────────────┼───────────────┐
        │ 180m          │ 200m          │ 170m
    [Node A] ←--→   [Node B] ←--→   [Node C]
   📸 ESP-NOW      📸 ESP-NOW      📸 ESP-NOW
   (2 hops)        (1 hop)         (1 hop)
        │               │               │
        │ 190m          │               │ 160m
    [Node D]            │           [Node E]
   📸 ESP-NOW           │          📸 ESP-NOW
   (3 hops)             │         (2 hops)
                        │ 220m
                   [Node F]
                  📸 ESP-NOW
                 (2 hops)
```

### Recommended Spacing
- **Node spacing**: 150-200m apart for reliable ESP-NOW connectivity
- **Maximum hops**: 5 hops to gateway (keeps latency reasonable)  
- **Total coverage**: 1000m radius from gateway
- **Gateway placement**: Highest point for maximum coverage

## Power Management

### Expected Battery Life
- **Sensor Nodes**: 2-6 months on 3000-5000mAh battery
- **Gateway Nodes**: Continuous operation with solar charging
- **Deep sleep current**: 0.1-0.5mA (99% of time)
- **Active transmission**: 50-100mA for 5-15 seconds per detection

### Solar Panel Sizing
- **Sensor Nodes**: 5-10W panel with 3000-5000mAh battery
- **Gateway Nodes**: 20W+ panel with 10000mAh+ battery bank
- **Charge controller**: MPPT recommended for efficiency

## API Documentation

### Gateway Registration
```http
POST /api/gateway/register
Content-Type: application/json

{
  "gateway_id": "HERD_GW_001",
  "zone": "North",
  "capabilities": ["TFLite", "ESP-NOW", "4G"],
  "status": "ACTIVE"
}
```

### Wildlife Detection Alert
```http
POST /api/wildlife-detection
Content-Type: application/json

{
  "node_id": "HERD_NODE_001",
  "gateway_id": "HERD_GW_001", 
  "timestamp": "2025-07-09T20:00:00Z",
  "species": "elephant",
  "confidence": 0.89,
  "is_wildlife": true,
  "image_data": "base64_encoded_image",
  "battery_voltage": 3.85,
  "rssi": -45,
  "hop_count": 2
}
```

### System Status
```http
GET /api/stats
```

## Troubleshooting

### Common Issues

1. **ESP-NOW Connection Failed**
   - Check MAC addresses in code
   - Verify both devices on same WiFi channel
   - Ensure ESP-NOW initialized before adding peers

2. **Image Transmission Timeout**
   - Reduce image quality/size in camera config
   - Check ESP-NOW range (max 200-300m in jungle)
   - Verify packet reassembly logic

3. **TFLite Model Not Loading**
   - Ensure model file in Android assets folder
   - Check model compatibility with TFLite runtime
   - Verify sufficient device memory

4. **Backend Connection Issues**
   - Check network connectivity
   - Verify API endpoints and authentication
   - Monitor server logs for errors

### Debug Commands

```bash
# Monitor ESP32 serial output
arduino-cli monitor -p /dev/ttyUSB0 -c baudrate=115200

# Check Android app logs
adb logcat | grep HERDGateway

# Test backend API
curl -X GET http://localhost:8000/api/health

# Monitor network traffic
tcpdump -i wlan0 port 80
```

## Performance Metrics

### Expected Performance
- **Detection latency**: 30-60 seconds from motion to alert
- **Image transmission**: 5-30 seconds depending on hops
- **TFLite processing**: 200-500ms per image
- **False positive rate**: <10% with trained model
- **Network reliability**: >95% packet delivery within range

### Monitoring Points
- Node battery levels
- Gateway connectivity status
- Image transmission success rate
- TFLite detection accuracy
- Backend API response times

## Security Considerations

1. **WiFi Security**: WPA2 encryption on all hotspots
2. **API Authentication**: JWT tokens for backend access
3. **Image Encryption**: TLS for image transmission
4. **Access Control**: Restrict backend API access
5. **Firmware Updates**: Secure OTA update mechanism

## Maintenance Schedule

### Weekly
- Check gateway connectivity
- Monitor battery levels via backend API
- Review detection accuracy

### Monthly  
- Physical inspection of solar panels
- Clean camera lenses
- Update TFLite model if needed

### Quarterly
- Battery replacement for low-performing nodes
- Firmware updates
- System performance review

## Future Enhancements

1. **Over-the-Air Updates**: Remote firmware updates via ESP-NOW
2. **Advanced Routing**: Dynamic mesh routing with load balancing
3. **Edge AI**: Deploy smaller models directly on ESP32
4. **Satellite Backup**: LoRa/satellite for remote areas
5. **Mobile App**: Flutter app for field technicians

---

This implementation provides a complete, production-ready wildlife detection system with automated image collection, real-time AI processing, and reliable mesh networking. The system eliminates dangerous manual data collection while providing real-time wildlife alerts to protect both humans and animals.
