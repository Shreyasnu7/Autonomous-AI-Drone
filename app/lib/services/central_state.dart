import 'dart:async';
import 'package:flutter/foundation.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'auth_service.dart';
import 'telemetry_service.dart';
import 'api_service.dart';

class CentralState extends ChangeNotifier {
  final TelemetryService telemetry = TelemetryService();
  
  bool isLoggedIn = false;
  Map<String, dynamic>? userProfile;
  String? connectedDroneId;

  Map<String, dynamic> telemetryData = {};
  
  // App Config (Persisted)
  Map<String, dynamic> config = {
      "units": "metric",
      "joystick_mode": 2,
      "sensitivity": 0.7,
      "expo": 0.3,
      "cam_source": "internal",
      "stream_res": "480p",
      "cap_res": "4k",
      "rth_alt": 50,
      "obstacle_avoidance": true,
      "vision_pos": true,
      "rth_behavior": "user", // "user" or "home"
      "rth_behavior": "user", // "user" or "home"
      "land_behavior": "user",
      "connection_mode": "tailscale", // cloud, local, tailscale, bluetooth (default: direct Tailscale)
      "local_ip": "192.168.4.1", // Default Radxa Hotspot IP
      "tailscale_ip": "100.64.0.30" // Radxa Tailscale IP (works across any network)
  };

  // Helper to update config and dispatch to Drone
  void updateConfig(String key, dynamic value) {
     config[key] = value;

     // BUGFIX: persist networking config so a typed IP survives an app restart
     if (key == 'local_ip' || key == 'tailscale_ip' || key == 'connection_mode') {
        SharedPreferences.getInstance().then((p) => p.setString('cfg_$key', value.toString()));
     }

     notifyListeners();
     
     // REAL COMMAND DISPATCH
     if (key == 'rth_alt') {
        // value is in meters, drone expects cm possibly? 
        // My set_rth_alt check in python expected 'alt' in cm
        ApiService.sendCommand("set_rth_alt", {"alt": (value * 100).toInt()});
     }

     if (key == 'batt_threshold') {
        ApiService.sendCommand("set_batt_threshold", {"threshold": value});
     }

     if (key == 'rth_behavior') {
        // Send to DirectorCore
        ApiService.sendCommand("SET_CONFIG:rth_behavior=$value", {});
     }

     if (key == 'land_behavior') {
        ApiService.sendCommand("SET_CONFIG:land_behavior=$value", {});
     }

     // NEW: Real Safety Toggle Propagation
     if (key == 'obstacle_avoidance' || key == 'vision_pos') {
        ApiService.sendCommand("set_safety_config", {
           "obstacle_avoidance": config['obstacle_avoidance'],
           "vision_pos": config['vision_pos']
        });
     }
     
     if (key == 'cap_res') {
        // DirectorCore expects SET_CONFIG:res=value
        ApiService.sendCommand("SET_CONFIG:res=$value", {});
     }

     if (key == 'stream_res') {
         // DirectorCore expects SET_CONFIG:stream_res=value
         ApiService.sendCommand("SET_CONFIG:stream_res=$value", {});
     }
     
     if (key == 'cam_source') {
         // DirectorCore expects SET_CONFIG:source=internal/external
         ApiService.sendCommand("SET_CONFIG:source=$value", {});
     }
  }

  // History
  final List<double> _altHistory = List.filled(50, 0.0, growable: true);
  final List<double> _speedHistory = List.filled(50, 0.0, growable: true);
  final List<double> _batHistory = List.filled(50, 0.0, growable: true);

  // Getters
  List<double> get altHistory => List.unmodifiable(_altHistory);
  List<double> get speedHistory => List.unmodifiable(_speedHistory);
  List<double> get batHistory => List.unmodifiable(_batHistory);

  // Helper to add history
  void _addToHistory(List<double> list, double val) {
     list.add(val);
     if (list.length > 50) list.removeAt(0);
  }

  // V110: Server status tracking
  bool serverOnline = false;

  // Getters for UI
  String get droneId => connectedDroneId ?? "Unknown Drone";
  bool get isDeviceConnected => connectedDroneId != null && telemetry.isConnected;
  double get currentBattery => (telemetryData['battery'] ?? 0).toDouble();
  double get droneLat => (telemetryData['lat'] ?? 0.0).toDouble();
  double get droneLng => (telemetryData['lng'] ?? 0.0).toDouble();

  // Init
  Future<void> init() async {
    await ApiService.loadToken();
    
    // Load Persistent Config & User Data
    final prefs = await SharedPreferences.getInstance();
    
    // 1. AI KEYS
    config['gemini_api_key'] = prefs.getString('api_key_gemini') ?? "";
    config['openai_api_key'] = prefs.getString('api_key_openai') ?? "";

    // BUGFIX: restore saved networking config (these used to revert to defaults on restart)
    config['local_ip'] = prefs.getString('cfg_local_ip') ?? config['local_ip'];
    config['tailscale_ip'] = prefs.getString('cfg_tailscale_ip') ?? config['tailscale_ip'];
    config['connection_mode'] = prefs.getString('cfg_connection_mode') ?? config['connection_mode'];
    
    
    // 2. AUTO-LOGIN CHECK
    if (ApiService.token != null && ApiService.token!.isNotEmpty) {
       print("Token found, auto-logging in...");
       isLoggedIn = true;
       userProfile = {
         "username": prefs.getString('cached_username') ?? "Commander",
         "email": prefs.getString('cached_email') ?? ""
       };
    }

    // V110: Auto-wake server on app start with retry
    _wakeServerWithRetry();

    // Listen to telemetry to update local state
    telemetry.telemetryStream.listen((data) {
       telemetryData = data;
       
       // Update History Buffers
       if (data.containsKey("altitude")) _addToHistory(_altHistory, (data["altitude"] as num).toDouble());
       if (data.containsKey("speed")) _addToHistory(_speedHistory, (data["speed"] as num).toDouble());
       if (data.containsKey("battery")) _addToHistory(_batHistory, (data["battery"] as num).toDouble());

       notifyListeners();
    });

    notifyListeners();
  }

  // V110: Wake server with retry (Render cold start takes 30-60s)
  Future<void> _wakeServerWithRetry() async {
    for (int attempt = 1; attempt <= 5; attempt++) {
      serverOnline = await ApiService.wakeUp();
      if (serverOnline) {
        print("✅ Server confirmed online (attempt $attempt)");
        notifyListeners();
        return;
      }
      print("⏳ Server not ready yet (attempt $attempt/5). Retrying in ${attempt * 3}s...");
      await Future.delayed(Duration(seconds: attempt * 3));
    }
    print("⚠️ Server may still be starting. Will connect when available.");
    notifyListeners();
  }

  Future<bool> login(String email, String password) async {
    try {
      userProfile = await AuthService.login(email, password);
      isLoggedIn = true;
      notifyListeners();
      return true;
    } catch (e) {
      print("Login Failed: $e");
      return false;
    }
  }

  Future<void> connectDevice(String id) async {
    // Determine connection type
    connectedDroneId = id;

    final mode = config['connection_mode'] ?? 'cloud';
    final ip = config['local_ip'] ?? '192.168.4.1';
    final tsIp = config['tailscale_ip'] ?? '100.64.0.30'; // Cubie A7Z Tailscale IP

    await telemetry.connect(
        id,
        mode: mode,
        localIp: ip,
        tailscaleIp: tsIp,
    );
    notifyListeners();
  }
  
  void logout() {
    AuthService.logout();
    isLoggedIn = false;
    userProfile = null;
    telemetry.disconnect();
    notifyListeners();
  }

  String formatSpeed(double ms) {
     if (config['units'] == 'imperial') {
       return "${(ms * 2.23694).toStringAsFixed(1)} mph";
     }
     return "${ms.toStringAsFixed(1)} m/s";
  }
  String formatDist(double m) {
     if (config['units'] == 'imperial') {
       return "${(m * 3.28084).toStringAsFixed(1)} ft";
     }
     return "${m.toStringAsFixed(1)} m";
  }
}
