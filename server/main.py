"""
HERD Wildlife Detection System - FastAPI Backend
Handles wildlife detection alerts, image processing, and push notifications
"""

from fastapi import FastAPI, HTTPException, UploadFile, File, Depends
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from pydantic import BaseModel, Field
from typing import List, Dict, Optional
import asyncio
import logging
from datetime import datetime, timedelta
import uuid
import base64
import json
import os
from pathlib import Path

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize FastAPI app
app = FastAPI(
    title="HERD Wildlife Detection API",
    description="Backend API for HERD jungle wildlife intrusion detection system",
    version="1.0.0"
)

# CORS middleware for frontend access
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Configure properly in production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Data Models
class GatewayInfo(BaseModel):
    gateway_id: str
    zone: str
    capabilities: List[str]
    status: str  # ACTIVE, OFFLINE, MAINTENANCE
    last_seen: datetime
    processed_detections: int = 0
    avg_response_time: float = 0.0
    priority: int = 1

class NodeInfo(BaseModel):
    node_id: str
    gateway_id: str
    last_seen: datetime
    battery_voltage: float
    rssi: int
    images_sent: int = 0
    is_active: bool = True

class WildlifeDetection(BaseModel):
    node_id: str
    gateway_id: str
    timestamp: datetime
    species: str
    confidence: float
    image_data: Optional[str] = None  # Base64 encoded
    image_url: Optional[str] = None
    bounding_box: Optional[List[int]] = None
    is_wildlife: bool
    processing_time_ms: int
    battery_voltage: float
    rssi: int
    hop_count: int = 0

class MotionAlert(BaseModel):
    node_id: str
    gateway_id: str
    timestamp: datetime
    battery_voltage: float
    rssi: int
    trigger_count: int = 1

class NotificationPayload(BaseModel):
    title: str
    body: str
    data: Dict
    image_url: Optional[str] = None

# In-memory storage (replace with proper database in production)
active_gateways: Dict[str, GatewayInfo] = {}
known_nodes: Dict[str, NodeInfo] = {}
wildlife_detections: List[WildlifeDetection] = []
motion_alerts: List[MotionAlert] = []
notification_subscribers: List[str] = []  # FCM tokens

# Configuration
IMAGES_DIR = Path("stored_images")
IMAGES_DIR.mkdir(exist_ok=True)
CDN_BASE_URL = "http://your-cdn.com/images"  # Configure your CDN
MAX_DETECTIONS_HISTORY = 1000

@app.on_event("startup")
async def startup_event():
    """Initialize the application"""
    logger.info("🚀 HERD Backend API starting up...")
    logger.info(f"📁 Images directory: {IMAGES_DIR.absolute()}")

@app.get("/")
async def root():
    """Root endpoint with API information"""
    return {
        "message": "HERD Wildlife Detection API",
        "version": "1.0.0",
        "status": "active",
        "endpoints": {
            "gateways": "/api/gateways",
            "nodes": "/api/nodes", 
            "detections": "/api/detections",
            "alerts": "/api/alerts"
        }
    }

@app.get("/api/health")
async def health_check():
    """Health check endpoint"""
    return {
        "status": "healthy",
        "timestamp": datetime.utcnow(),
        "active_gateways": len(active_gateways),
        "known_nodes": len(known_nodes),
        "total_detections": len(wildlife_detections)
    }

# Gateway Management Endpoints
@app.post("/api/gateway/register")
async def register_gateway(gateway: GatewayInfo):
    """Register a new gateway or update existing one"""
    gateway.last_seen = datetime.utcnow()
    active_gateways[gateway.gateway_id] = gateway
    
    logger.info(f"✅ Gateway {gateway.gateway_id} registered for zone {gateway.zone}")
    return {"status": "registered", "gateway_id": gateway.gateway_id}

@app.post("/api/gateway/heartbeat/{gateway_id}")
async def gateway_heartbeat(gateway_id: str):
    """Update gateway last seen timestamp"""
    if gateway_id in active_gateways:
        active_gateways[gateway_id].last_seen = datetime.utcnow()
        active_gateways[gateway_id].status = "ACTIVE"
        logger.debug(f"💓 Heartbeat from gateway {gateway_id}")
    else:
        raise HTTPException(status_code=404, detail="Gateway not found")
    
    return {"status": "heartbeat_received"}

@app.get("/api/gateways")
async def get_gateways():
    """Get all registered gateways with their status"""
    gateway_list = []
    current_time = datetime.utcnow()
    
    for gateway_id, gateway in active_gateways.items():
        time_since_last_seen = (current_time - gateway.last_seen).total_seconds()
        status = "ACTIVE" if time_since_last_seen < 300 else "OFFLINE"  # 5 minutes timeout
        
        gateway_info = {
            "gateway_id": gateway_id,
            "zone": gateway.zone,
            "status": status,
            "last_seen": gateway.last_seen,
            "processed_detections": gateway.processed_detections,
            "avg_response_time": gateway.avg_response_time,
            "priority": gateway.priority,
            "seconds_since_last_seen": int(time_since_last_seen)
        }
        gateway_list.append(gateway_info)
    
    active_count = sum(1 for g in gateway_list if g["status"] == "ACTIVE")
    
    return {
        "gateways": gateway_list,
        "total_gateways": len(gateway_list),
        "active_gateways": active_count
    }

# Node Management Endpoints
@app.post("/api/node/register")
async def register_node(node: NodeInfo):
    """Register or update a sensor node"""
    node.last_seen = datetime.utcnow()
    known_nodes[node.node_id] = node
    
    logger.info(f"📡 Node {node.node_id} registered via gateway {node.gateway_id}")
    return {"status": "registered", "node_id": node.node_id}

@app.get("/api/nodes")
async def get_nodes():
    """Get all known sensor nodes with their status"""
    node_list = []
    current_time = datetime.utcnow()
    
    for node_id, node in known_nodes.items():
        time_since_last_seen = (current_time - node.last_seen).total_seconds()
        is_active = time_since_last_seen < 900  # 15 minutes timeout
        
        node_info = {
            "node_id": node_id,
            "gateway_id": node.gateway_id,
            "last_seen": node.last_seen,
            "battery_voltage": node.battery_voltage,
            "rssi": node.rssi,
            "images_sent": node.images_sent,
            "is_active": is_active,
            "minutes_since_last_seen": int(time_since_last_seen / 60)
        }
        node_list.append(node_info)
    
    active_count = sum(1 for n in node_list if n["is_active"])
    
    return {
        "nodes": node_list,
        "total_nodes": len(node_list),
        "active_nodes": active_count
    }

# Motion Alert Endpoints
@app.post("/api/motion-alert")
async def receive_motion_alert(alert: MotionAlert):
    """Receive motion detection alert from gateway"""
    # Store motion alert
    motion_alerts.append(alert)
    
    # Update node information
    if alert.node_id in known_nodes:
        known_nodes[alert.node_id].last_seen = alert.timestamp
        known_nodes[alert.node_id].battery_voltage = alert.battery_voltage
        known_nodes[alert.node_id].rssi = alert.rssi
    
    logger.info(f"🎯 Motion alert from {alert.node_id} via {alert.gateway_id}")
    
    # Keep only recent alerts
    if len(motion_alerts) > MAX_DETECTIONS_HISTORY:
        motion_alerts.pop(0)
    
    return {"status": "alert_received", "alert_id": str(uuid.uuid4())}

# Wildlife Detection Endpoints
@app.post("/api/wildlife-detection")
async def receive_wildlife_detection(detection: WildlifeDetection):
    """Receive confirmed wildlife detection from gateway"""
    
    # Validate gateway
    if detection.gateway_id not in active_gateways:
        logger.warning(f"⚠️ Unknown gateway: {detection.gateway_id}")
        # Still process but log warning
    
    # Check for duplicate detections (same node, similar timestamp)
    recent_threshold = datetime.utcnow() - timedelta(minutes=5)
    duplicate = any(
        d.node_id == detection.node_id and 
        d.timestamp >= recent_threshold and 
        d.is_wildlife
        for d in wildlife_detections[-10:]  # Check last 10 detections
    )
    
    if duplicate:
        logger.warning(f"⚠️ Duplicate detection filtered: {detection.node_id}")
        return {"status": "duplicate_filtered"}
    
    # Store detection
    wildlife_detections.append(detection)
    
    # Update gateway stats
    if detection.gateway_id in active_gateways:
        gateway = active_gateways[detection.gateway_id]
        gateway.processed_detections += 1
        if gateway.avg_response_time == 0:
            gateway.avg_response_time = detection.processing_time_ms
        else:
            gateway.avg_response_time = (gateway.avg_response_time + detection.processing_time_ms) / 2
    
    # Update node information
    if detection.node_id in known_nodes:
        node = known_nodes[detection.node_id]
        node.last_seen = detection.timestamp
        node.battery_voltage = detection.battery_voltage
        node.rssi = detection.rssi
        node.images_sent += 1
    
    # Store image if provided
    image_url = None
    if detection.image_data:
        image_url = await store_detection_image(detection)
        detection.image_url = image_url
    
    # Send push notification for confirmed wildlife
    alert_sent = False
    if detection.is_wildlife and detection.confidence > 0.7:
        alert_sent = await send_wildlife_alert_notification(detection)
        logger.info(f"🐘 Wildlife alert sent: {detection.species} detected by {detection.gateway_id}")
    
    # Keep only recent detections
    if len(wildlife_detections) > MAX_DETECTIONS_HISTORY:
        wildlife_detections.pop(0)
    
    return {
        "status": "detection_processed", 
        "alert_sent": alert_sent,
        "detection_id": str(uuid.uuid4()),
        "image_url": image_url
    }

@app.get("/api/detections")
async def get_detections(
    limit: int = 50,
    wildlife_only: bool = False,
    since: Optional[datetime] = None
):
    """Get recent wildlife detections"""
    filtered_detections = wildlife_detections
    
    # Filter by wildlife only
    if wildlife_only:
        filtered_detections = [d for d in filtered_detections if d.is_wildlife]
    
    # Filter by timestamp
    if since:
        filtered_detections = [d for d in filtered_detections if d.timestamp >= since]
    
    # Sort by timestamp (newest first) and limit
    sorted_detections = sorted(filtered_detections, key=lambda x: x.timestamp, reverse=True)
    limited_detections = sorted_detections[:limit]
    
    # Convert to dict format for JSON response
    detection_list = []
    for detection in limited_detections:
        detection_dict = detection.dict()
        # Remove large image_data for list response
        if 'image_data' in detection_dict:
            del detection_dict['image_data']
        detection_list.append(detection_dict)
    
    return {
        "detections": detection_list,
        "total_count": len(filtered_detections),
        "returned_count": len(detection_list),
        "wildlife_count": len([d for d in wildlife_detections if d.is_wildlife])
    }

@app.get("/api/detections/{detection_id}")
async def get_detection_detail(detection_id: str):
    """Get detailed information for a specific detection"""
    # In a real implementation, you'd look up by actual ID
    # For now, return the most recent detection
    if not wildlife_detections:
        raise HTTPException(status_code=404, detail="No detections found")
    
    latest_detection = wildlife_detections[-1]
    return latest_detection.dict()

# Statistics Endpoints
@app.get("/api/stats")
async def get_system_statistics():
    """Get overall system statistics"""
    current_time = datetime.utcnow()
    
    # Calculate statistics
    total_detections = len(wildlife_detections)
    wildlife_detections_count = len([d for d in wildlife_detections if d.is_wildlife])
    false_positives = total_detections - wildlife_detections_count
    
    # Recent activity (last 24 hours)
    last_24h = current_time - timedelta(hours=24)
    recent_detections = [d for d in wildlife_detections if d.timestamp >= last_24h]
    recent_wildlife = [d for d in recent_detections if d.is_wildlife]
    
    # Gateway statistics
    active_gateway_count = len([g for g in active_gateways.values() 
                               if (current_time - g.last_seen).total_seconds() < 300])
    
    # Node statistics
    active_node_count = len([n for n in known_nodes.values() 
                           if (current_time - n.last_seen).total_seconds() < 900])
    
    return {
        "system_overview": {
            "total_detections": total_detections,
            "wildlife_detections": wildlife_detections_count,
            "false_positives": false_positives,
            "accuracy_rate": (wildlife_detections_count / max(total_detections, 1)) * 100
        },
        "recent_activity": {
            "detections_24h": len(recent_detections),
            "wildlife_24h": len(recent_wildlife),
            "motion_alerts_24h": len([a for a in motion_alerts if a.timestamp >= last_24h])
        },
        "network_status": {
            "active_gateways": active_gateway_count,
            "total_gateways": len(active_gateways),
            "active_nodes": active_node_count,
            "total_nodes": len(known_nodes)
        },
        "timestamp": current_time
    }

# Notification Management
@app.post("/api/notifications/subscribe")
async def subscribe_to_notifications(token: str):
    """Subscribe to push notifications"""
    if token not in notification_subscribers:
        notification_subscribers.append(token)
        logger.info(f"📱 New notification subscriber: {token[:10]}...")
    
    return {"status": "subscribed", "subscribers_count": len(notification_subscribers)}

@app.post("/api/notifications/unsubscribe") 
async def unsubscribe_from_notifications(token: str):
    """Unsubscribe from push notifications"""
    if token in notification_subscribers:
        notification_subscribers.remove(token)
        logger.info(f"📱 Notification unsubscribed: {token[:10]}...")
    
    return {"status": "unsubscribed", "subscribers_count": len(notification_subscribers)}

# Image Management
@app.post("/api/images/upload")
async def upload_image(file: UploadFile = File(...)):
    """Upload detection image"""
    if not file.content_type.startswith('image/'):
        raise HTTPException(status_code=400, detail="File must be an image")
    
    # Generate unique filename
    timestamp = datetime.utcnow().strftime("%Y%m%d_%H%M%S")
    filename = f"{timestamp}_{file.filename}"
    file_path = IMAGES_DIR / filename
    
    # Save file
    with open(file_path, "wb") as buffer:
        content = await file.read()
        buffer.write(content)
    
    # Generate URL
    image_url = f"{CDN_BASE_URL}/{filename}"
    
    logger.info(f"📷 Image uploaded: {filename} ({len(content)} bytes)")
    
    return {
        "filename": filename,
        "url": image_url,
        "size": len(content)
    }

# Helper Functions
async def store_detection_image(detection: WildlifeDetection) -> Optional[str]:
    """Store base64 image data to file and return URL"""
    if not detection.image_data:
        return None
    
    try:
        # Decode base64 image
        image_bytes = base64.b64decode(detection.image_data)
        
        # Generate filename
        timestamp = detection.timestamp.strftime("%Y%m%d_%H%M%S")
        filename = f"{detection.node_id}_{timestamp}_{detection.species}.jpg"
        file_path = IMAGES_DIR / filename
        
        # Save image
        with open(file_path, "wb") as f:
            f.write(image_bytes)
        
        # Generate URL
        image_url = f"{CDN_BASE_URL}/{filename}"
        
        logger.info(f"📷 Detection image stored: {filename} ({len(image_bytes)} bytes)")
        return image_url
        
    except Exception as e:
        logger.error(f"❌ Failed to store image: {e}")
        return None

async def send_wildlife_alert_notification(detection: WildlifeDetection) -> bool:
    """Send push notification for wildlife detection"""
    if not notification_subscribers:
        logger.warning("📱 No notification subscribers")
        return False
    
    try:
        # Get gateway info for zone
        gateway = active_gateways.get(detection.gateway_id, {})
        zone = getattr(gateway, 'zone', 'Unknown') if hasattr(gateway, 'zone') else 'Unknown'
        
        # Create notification payload
        notification = NotificationPayload(
            title=f"🐘 {detection.species.title()} Alert!",
            body=f"Wildlife detected in {zone} zone (Confidence: {detection.confidence:.1%})",
            data={
                "node_id": detection.node_id,
                "gateway_id": detection.gateway_id,
                "zone": zone,
                "species": detection.species,
                "confidence": detection.confidence,
                "timestamp": detection.timestamp.isoformat(),
                "image_url": detection.image_url
            },
            image_url=detection.image_url
        )
        
        # In a real implementation, send via FCM
        # For now, just log the notification
        logger.info(f"📱 Notification sent to {len(notification_subscribers)} subscribers")
        logger.info(f"   Title: {notification.title}")
        logger.info(f"   Body: {notification.body}")
        
        return True
        
    except Exception as e:
        logger.error(f"❌ Failed to send notification: {e}")
        return False

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8000,
        reload=True,
        log_level="info"
    )
