import 'dart:async';
import 'dart:convert';
import 'dart:math';
import 'package:web_socket_channel/web_socket_channel.dart';
import 'package:web_socket_channel/status.dart' as status;
import 'package:geolocator/geolocator.dart'; 
import 'api_service.dart';

class TelemetryService {
  // Configured dynamically from ApiService
  
  WebSocketChannel? _channel;
  final _telemetryController = StreamController<Map<String, dynamic>>.broadcast();
  Stream<Map<String, dynamic>> get telemetryStream => _telemetryController.stream;

  final _alertController = StreamController<Map<String, dynamic>>.broadcast();
  Stream<Map<String, dynamic>> get alertStream => _alertController.stream;

  // AI director's reasoning/decision pushed back from the laptop over Tailscale.
  final _aiResponseController = StreamController<Map<String, dynamic>>.broadcast();
  Stream<Map<String, dynamic>> get aiResponseStream => _aiResponseController.stream;

  bool _isConnected = false;
  String? _currentUrl;   // target we're currently linked to (detects mode/IP switches)
  Timer? _locationTimer;

  Future<void> connect(String droneId, {String mode = 'cloud', String localIp = '192.168.4.1', String tailscaleIp = '100.64.0.30'}) async {
    // Build the target URL FIRST so we can detect a mode/IP switch
    String wsUrl;
    if (mode == 'tailscale') {
       wsUrl = "ws://$tailscaleIp:8000";
    } else if (mode == 'local') {
       wsUrl = "ws://$localIp:8000";
    } else {
       final baseUrl = ApiService.baseUrl;
       wsUrl = baseUrl.startsWith('https')
          ? baseUrl.replaceFirst('https', 'wss')
          : baseUrl.replaceFirst('http', 'ws');
       wsUrl = "$wsUrl/ws/connect/app_client";
    }

    // Same target already linked? keep it (avoids churn).
    // Different target (e.g. switched Cloud -> Tailscale -> Local)? tear down and reconnect.
    if (_channel != null && _isConnected && _currentUrl == wsUrl) return;
    try { await _channel?.sink.close(); } catch (_) {}
    _isConnected = false;
    _currentUrl = wsUrl;

    try {
      print("🔗 Connecting ($mode): $wsUrl");
      _channel = WebSocketChannel.connect(Uri.parse(wsUrl));
      _isConnected = true;
      
      // Auth handshake (Required for Cloud, ignored/safe for Local)
      _channel!.sink.add(jsonEncode({"type": "auth", "token": "bearer_token"}));
      _channel!.sink.add(jsonEncode({"type": "connect_drone", "droneId": droneId}));

      // Start streaming user location immediately on connection
      _startLocationStream();

      _channel!.stream.listen(
        (message) {
          try {
            final data = jsonDecode(message);
            if (data['type'] == 'telemetry') {
              _telemetryController.add(data['payload']);
            } else if (data['type'] == 'alert') {
              _alertController.add(data['payload']);
            } else if (data['type'] == 'ai_response' || data['type'] == 'ai_status') {
              _aiResponseController.add(Map<String, dynamic>.from(data['payload'] ?? {}));
            }
          } catch (e) {
            print("Telemetery Parse Error: $e");
          }
        },
        onDone: () {
           print("WS Closed.");
           disconnect();
        },
        onError: (e) {
           print("WS Error: $e");
           disconnect();
        },
      );
    } catch (e) {
      print("WS Connection Failed: $e");
      _isConnected = false;
    }
  }

  bool get isConnected => _isConnected;

  // Logic to get permissions and stream location
  void _startLocationStream() async {
    bool serviceEnabled;
    LocationPermission permission;

    serviceEnabled = await Geolocator.isLocationServiceEnabled();
    if (!serviceEnabled) {
      return Future.error('Location services are disabled.');
    }

    permission = await Geolocator.checkPermission();
    if (permission == LocationPermission.denied) {
      permission = await Geolocator.requestPermission();
      if (permission == LocationPermission.denied) return;
    }

    if (permission == LocationPermission.deniedForever) return;

    // Send location every 2 seconds
    _locationTimer?.cancel();
    _locationTimer = Timer.periodic(Duration(seconds: 2), (timer) async {
       if (!_isConnected) {
         timer.cancel(); 
         return;
       }
       Position position = await Geolocator.getCurrentPosition(desiredAccuracy: LocationAccuracy.high);
       sendUserLocation(position.latitude, position.longitude);
    });
  }

  void sendJoystick(double x, double y, double z, double r) {
    if (_isConnected && _channel != null) {
      _channel!.sink.add(jsonEncode({
        "type": "joystick",
        "payload": {"x": x, "y": y, "z": z, "r": r}
      }));
    }
  }

  // Alias for legacy calls
  void sendControl(double x, double y, double z, double r) => sendJoystick(x, y, z, r);
  
  void sendMission(List<dynamic> waypoints) {
    if (_isConnected && _channel != null) {
      _channel!.sink.add(jsonEncode({
        "type": "mission",
        "payload": waypoints
      }));
    }
  }

  void sendUserLocation(double lat, double lng) {
    if (_isConnected && _channel != null) {
      _channel!.sink.add(jsonEncode({
        "type": "user_gps",
        "payload": {"lat": lat, "lng": lng}
      }));
    }
  }

  void sendCommand(String cmd, [Map<String, dynamic>? payload]) {
    if (_isConnected && _channel != null) {
      // V110: Send command as type directly for bridge compatibility
      // Bridge extracts real command from type:"command" payload:"LAND"
      // But also works if we send type:"LAND" directly
      _channel!.sink.add(jsonEncode({
        "type": "command",
        "payload": payload != null ? {"command": cmd, "payload": payload} : cmd
      }));
    }
  }

  /// Send a typed command with payload (for settings, config, etc.)
  void sendTypedCommand(String type, Map<String, dynamic> payload) {
    if (_isConnected && _channel != null) {
      _channel!.sink.add(jsonEncode({
        "type": type,
        "payload": payload
      }));
    }
  }

  /// Send an AI job DIRECT over Tailscale to the laptop AI (bridge relays type 'ai_job'
  /// to laptop_vision). Returns true if it was sent over the live WS, false otherwise
  /// (caller falls back to cloud HTTP). The laptop's process_job reads these top-level fields.
  bool sendAiJob(Map<String, dynamic> p) {
    if (_isConnected && _channel != null) {
      _channel!.sink.add(jsonEncode({
        "type": "ai_job",
        "text": p["text"] ?? "",
        "payload": p["text"] ?? "",
        "job_id": "app_${DateTime.now().millisecondsSinceEpoch}",
        "user_id": "app",
        "drone_id": "drone1",
        "provider": p["provider"] ?? "gemini",
        "api_keys": p["api_keys"] ?? {},
        "images": p["media"] ?? [],
      }));
      return true;
    }
    return false;
  }

  void disconnect() {
    _locationTimer?.cancel();
    // 1000 (normalClosure) is valid; 1001 (goingAway) is FORBIDDEN by Dart and was crashing connect()
    try { _channel?.sink.close(status.normalClosure); } catch (_) {}
    _isConnected = false;
  }
}
