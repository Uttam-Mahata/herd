package com.herd.gateway;

import android.app.Service;
import android.content.Intent;
import android.os.IBinder;
import android.util.Log;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import java.io.*;
import java.net.*;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import org.tensorflow.lite.Interpreter;
import org.json.JSONObject;
import okhttp3.*;

/**
 * HERD Gateway Service - TFLite Wildlife Detection
 * Runs continuously to process images from ESP32 gateway
 * Communicates with ESP32 via HTTP and processes images with TensorFlow Lite
 */
public class TFLiteGatewayService extends Service {
    private static final String TAG = "HERDGateway";
    private static final String GATEWAY_ID = android.os.Build.MODEL + "_" + android.provider.Settings.Secure.ANDROID_ID;
    
    // Network configuration
    private static final String ESP32_IP = "192.168.4.1";  // ESP32 gateway IP
    private static final int HTTP_PORT = 8080;
    private static final String BACKEND_URL = "http://your-server.com/api";
    
    // TensorFlow Lite
    private Interpreter tflite;
    private boolean isModelLoaded = false;
    
    // HTTP server for ESP32 communication
    private ServerSocket serverSocket;
    private boolean isRunning = false;
    private ExecutorService executorService;
    
    // HTTP client for backend communication
    private OkHttpClient httpClient;
    
    @Override
    public void onCreate() {
        super.onCreate();
        Log.i(TAG, "🏰 HERD Gateway Service starting...");
        
        // Initialize components
        initializeTFLite();
        initializeHttpClient();
        startHttpServer();
        registerWithBackend();
        
        Log.i(TAG, "✅ Gateway Service initialized");
    }
    
    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {
        Log.i(TAG, "Gateway Service started");
        return START_STICKY; // Restart if killed
    }
    
    @Override
    public void onDestroy() {
        super.onDestroy();
        isRunning = false;
        
        if (serverSocket != null && !serverSocket.isClosed()) {
            try {
                serverSocket.close();
            } catch (IOException e) {
                Log.e(TAG, "Error closing server socket", e);
            }
        }
        
        if (executorService != null) {
            executorService.shutdown();
        }
        
        Log.i(TAG, "Gateway Service destroyed");
    }
    
    @Override
    public IBinder onBind(Intent intent) {
        return null; // Not a bound service
    }
    
    private void initializeTFLite() {
        try {
            // Load TFLite model from assets
            tflite = new Interpreter(loadModelFile("wildlife_detection.tflite"));
            isModelLoaded = true;
            Log.i(TAG, "✅ TFLite model loaded successfully");
        } catch (Exception e) {
            Log.e(TAG, "❌ Failed to load TFLite model", e);
            isModelLoaded = false;
        }
    }
    
    private java.nio.MappedByteBuffer loadModelFile(String modelPath) throws IOException {
        AssetFileDescriptor fileDescriptor = getAssets().openFd(modelPath);
        FileInputStream inputStream = new FileInputStream(fileDescriptor.getFileDescriptor());
        FileChannel fileChannel = inputStream.getChannel();
        long startOffset = fileDescriptor.getStartOffset();
        long declaredLength = fileDescriptor.getDeclaredLength();
        return fileChannel.map(FileChannel.MapMode.READ_ONLY, startOffset, declaredLength);
    }
    
    private void initializeHttpClient() {
        httpClient = new OkHttpClient.Builder()
            .connectTimeout(30, java.util.concurrent.TimeUnit.SECONDS)
            .writeTimeout(30, java.util.concurrent.TimeUnit.SECONDS)
            .readTimeout(30, java.util.concurrent.TimeUnit.SECONDS)
            .build();
    }
    
    private void startHttpServer() {
        executorService = Executors.newCachedThreadPool();
        
        executorService.execute(() -> {
            try {
                serverSocket = new ServerSocket(HTTP_PORT);
                isRunning = true;
                
                Log.i(TAG, "🌐 HTTP server started on port " + HTTP_PORT);
                
                while (isRunning && !serverSocket.isClosed()) {
                    try {
                        Socket clientSocket = serverSocket.accept();
                        executorService.execute(() -> handleClientConnection(clientSocket));
                    } catch (IOException e) {
                        if (isRunning) {
                            Log.e(TAG, "Error accepting client connection", e);
                        }
                    }
                }
            } catch (IOException e) {
                Log.e(TAG, "❌ Failed to start HTTP server", e);
            }
        });
    }
    
    private void handleClientConnection(Socket clientSocket) {
        try (BufferedReader reader = new BufferedReader(new InputStreamReader(clientSocket.getInputStream()));
             PrintWriter writer = new PrintWriter(clientSocket.getOutputStream(), true)) {
            
            // Read HTTP request
            String requestLine = reader.readLine();
            if (requestLine == null) return;
            
            Log.d(TAG, "📥 HTTP Request: " + requestLine);
            
            // Parse HTTP request
            String[] parts = requestLine.split(" ");
            if (parts.length >= 2) {
                String method = parts[0];
                String path = parts[1];
                
                if ("POST".equals(method) && "/api/new-image".equals(path)) {
                    handleNewImageRequest(reader, writer);
                } else if ("GET".equals(method) && "/api/status".equals(path)) {
                    handleStatusRequest(writer);
                } else {
                    sendHttpResponse(writer, 404, "Not Found", "{}");
                }
            }
            
        } catch (Exception e) {
            Log.e(TAG, "Error handling client connection", e);
        } finally {
            try {
                clientSocket.close();
            } catch (IOException e) {
                Log.e(TAG, "Error closing client socket", e);
            }
        }
    }
    
    private void handleNewImageRequest(BufferedReader reader, PrintWriter writer) {
        try {
            // Read HTTP headers
            String line;
            int contentLength = 0;
            
            while ((line = reader.readLine()) != null && !line.isEmpty()) {
                if (line.startsWith("Content-Length:")) {
                    contentLength = Integer.parseInt(line.substring(15).trim());
                }
            }
            
            // Read JSON body
            StringBuilder jsonBody = new StringBuilder();
            for (int i = 0; i < contentLength; i++) {
                jsonBody.append((char) reader.read());
            }
            
            // Parse JSON
            JSONObject imageInfo = new JSONObject(jsonBody.toString());
            String nodeId = imageInfo.getString("node_id");
            String filename = imageInfo.getString("filename");
            long timestamp = imageInfo.getLong("timestamp");
            
            Log.i(TAG, "📸 Processing new image from " + nodeId + ": " + filename);
            
            // Get image from ESP32
            byte[] imageData = getImageFromESP32(filename);
            
            if (imageData != null) {
                // Process with TFLite
                DetectionResult result = processImageWithTFLite(imageData, nodeId, timestamp);
                
                // Send result back to ESP32
                JSONObject response = new JSONObject();
                response.put("status", "processed");
                response.put("is_wildlife", result.isWildlife);
                response.put("confidence", result.confidence);
                response.put("species", result.species);
                response.put("processing_time_ms", result.processingTimeMs);
                
                sendHttpResponse(writer, 200, "OK", response.toString());
                
                // Send to backend if wildlife detected
                if (result.isWildlife && result.confidence > 0.7) {
                    sendDetectionToBackend(result, imageData);
                }
                
            } else {
                sendHttpResponse(writer, 500, "Internal Server Error", "{\"error\":\"Failed to get image\"}");
            }
            
        } catch (Exception e) {
            Log.e(TAG, "❌ Error processing image request", e);
            sendHttpResponse(writer, 500, "Internal Server Error", "{\"error\":\"Processing failed\"}");
        }
    }
    
    private void handleStatusRequest(PrintWriter writer) {
        try {
            JSONObject status = new JSONObject();
            status.put("gateway_id", GATEWAY_ID);
            status.put("status", "active");
            status.put("model_loaded", isModelLoaded);
            status.put("uptime_ms", System.currentTimeMillis());
            status.put("free_memory", Runtime.getRuntime().freeMemory());
            
            sendHttpResponse(writer, 200, "OK", status.toString());
        } catch (Exception e) {
            Log.e(TAG, "Error sending status", e);
        }
    }
    
    private void sendHttpResponse(PrintWriter writer, int statusCode, String statusText, String body) {
        writer.println("HTTP/1.1 " + statusCode + " " + statusText);
        writer.println("Content-Type: application/json");
        writer.println("Content-Length: " + body.length());
        writer.println("Access-Control-Allow-Origin: *");
        writer.println();
        writer.println(body);
        writer.flush();
    }
    
    private byte[] getImageFromESP32(String filename) {
        try {
            // Get image from ESP32 gateway
            String url = "http://" + ESP32_IP + "/api/image?filename=" + filename;
            
            Request request = new Request.Builder()
                .url(url)
                .build();
            
            try (Response response = httpClient.newCall(request).execute()) {
                if (response.isSuccessful() && response.body() != null) {
                    return response.body().bytes();
                }
            }
        } catch (Exception e) {
            Log.e(TAG, "❌ Failed to get image from ESP32", e);
        }
        return null;
    }
    
    private DetectionResult processImageWithTFLite(byte[] imageData, String nodeId, long timestamp) {
        long startTime = System.currentTimeMillis();
        
        DetectionResult result = new DetectionResult();
        result.nodeId = nodeId;
        result.timestamp = timestamp;
        result.gatewayId = GATEWAY_ID;
        
        if (!isModelLoaded) {
            Log.w(TAG, "⚠️ TFLite model not loaded");
            result.isWildlife = false;
            result.confidence = 0.0f;
            result.species = "unknown";
            return result;
        }
        
        try {
            // Decode and preprocess image
            Bitmap bitmap = BitmapFactory.decodeByteArray(imageData, 0, imageData.length);
            if (bitmap == null) {
                Log.w(TAG, "⚠️ Failed to decode image");
                result.isWildlife = false;
                return result;
            }
            
            // Resize image to model input size (e.g., 224x224)
            Bitmap resizedBitmap = Bitmap.createScaledBitmap(bitmap, 224, 224, true);
            
            // Convert to float array
            float[][][][] input = new float[1][224][224][3];
            for (int y = 0; y < 224; y++) {
                for (int x = 0; x < 224; x++) {
                    int pixel = resizedBitmap.getPixel(x, y);
                    input[0][y][x][0] = ((pixel >> 16) & 0xFF) / 255.0f; // R
                    input[0][y][x][1] = ((pixel >> 8) & 0xFF) / 255.0f;  // G
                    input[0][y][x][2] = (pixel & 0xFF) / 255.0f;         // B
                }
            }
            
            // Run inference
            float[][] output = new float[1][3]; // [background, elephant, tiger]
            tflite.run(input, output);
            
            // Parse results
            float elephantConf = output[0][1];
            float tigerConf = output[0][2];
            float maxConf = Math.max(elephantConf, tigerConf);
            
            result.confidence = maxConf;
            result.isWildlife = maxConf > 0.7f;
            result.species = elephantConf > tigerConf ? "elephant" : "tiger";
            result.processingTimeMs = System.currentTimeMillis() - startTime;
            
            Log.i(TAG, String.format("🤖 Detection result: %s (%.2f confidence)", 
                result.species, result.confidence));
            
        } catch (Exception e) {
            Log.e(TAG, "❌ TFLite inference failed", e);
            result.isWildlife = false;
            result.confidence = 0.0f;
            result.species = "error";
        }
        
        result.processingTimeMs = System.currentTimeMillis() - startTime;
        return result;
    }
    
    private void sendDetectionToBackend(DetectionResult result, byte[] imageData) {
        executorService.execute(() -> {
            try {
                // Create JSON payload
                JSONObject detection = new JSONObject();
                detection.put("node_id", result.nodeId);
                detection.put("gateway_id", result.gatewayId);
                detection.put("timestamp", new java.util.Date(result.timestamp).toString());
                detection.put("species", result.species);
                detection.put("confidence", result.confidence);
                detection.put("is_wildlife", result.isWildlife);
                detection.put("processing_time_ms", result.processingTimeMs);
                detection.put("image_data", android.util.Base64.encodeToString(imageData, android.util.Base64.DEFAULT));
                detection.put("battery_voltage", 3.7); // Get from ESP32
                detection.put("rssi", -45); // Get from ESP32
                detection.put("hop_count", 1);
                
                // Send to backend
                RequestBody body = RequestBody.create(
                    detection.toString(),
                    MediaType.get("application/json; charset=utf-8")
                );
                
                Request request = new Request.Builder()
                    .url(BACKEND_URL + "/wildlife-detection")
                    .post(body)
                    .build();
                
                try (Response response = httpClient.newCall(request).execute()) {
                    if (response.isSuccessful()) {
                        Log.i(TAG, "✅ Detection sent to backend");
                    } else {
                        Log.w(TAG, "⚠️ Backend response: " + response.code());
                    }
                }
                
            } catch (Exception e) {
                Log.e(TAG, "❌ Failed to send detection to backend", e);
            }
        });
    }
    
    private void registerWithBackend() {
        executorService.execute(() -> {
            try {
                JSONObject registration = new JSONObject();
                registration.put("gateway_id", GATEWAY_ID);
                registration.put("zone", "North"); // Configure per gateway
                registration.put("capabilities", new JSONArray().put("TFLite").put("ESP-NOW").put("4G"));
                registration.put("status", "ACTIVE");
                registration.put("last_seen", new java.util.Date().toString());
                
                RequestBody body = RequestBody.create(
                    registration.toString(),
                    MediaType.get("application/json; charset=utf-8")
                );
                
                Request request = new Request.Builder()
                    .url(BACKEND_URL + "/gateway/register")
                    .post(body)
                    .build();
                
                try (Response response = httpClient.newCall(request).execute()) {
                    if (response.isSuccessful()) {
                        Log.i(TAG, "✅ Gateway registered with backend");
                    } else {
                        Log.w(TAG, "⚠️ Gateway registration failed: " + response.code());
                    }
                }
                
            } catch (Exception e) {
                Log.e(TAG, "❌ Failed to register with backend", e);
            }
        });
    }
    
    // Detection result class
    public static class DetectionResult {
        public String nodeId;
        public String gatewayId;
        public long timestamp;
        public boolean isWildlife;
        public float confidence;
        public String species;
        public long processingTimeMs;
    }
}
