import 'dart:convert';
import 'package:http/http.dart' as http;
import 'package:shared_preferences/shared_preferences.dart';

class ApiService {
  // No Mock Data Allowed
 
  static const String baseUrl = "https://drone-server-r0qe.onrender.com";
  // Note: No /api/v1 prefix based on inspection of main.py
  
  static String? _token;
  static String? get token => _token; // Public Getter

  static Future<void> loadToken() async {
    final prefs = await SharedPreferences.getInstance();
    _token = prefs.getString('auth_token');
  } 

  static Future<void> setToken(String token) => saveToken(token); // Alias for compatibility

  static Future<void> saveToken(String token) async {
    _token = token;
    final prefs = await SharedPreferences.getInstance();
    await prefs.setString('auth_token', token);
  }

  static Future<void> clearToken() async {
    _token = null;
    final prefs = await SharedPreferences.getInstance();
    await prefs.remove('auth_token');
  }

  // Generic GET
  static Future<dynamic> get(String endpoint) async {


    try {
      final response = await http.get(
        Uri.parse('$baseUrl$endpoint'),
        headers: _headers,
      ).timeout(const Duration(seconds: 30));
      return _processResponse(response);
    } catch (e) {
      throw Exception("Network Error: $e");
    }
  }

  // Generic POST
  static Future<dynamic> post(String endpoint, Map<String, dynamic> body) async {


    try {
      final response = await http.post(
        Uri.parse('$baseUrl$endpoint'),
        headers: _headers,
        body: jsonEncode(body),
      ).timeout(const Duration(seconds: 30));
      return _processResponse(response);
    } catch (e) {
      throw Exception("Network Error: $e");
    }
  }

  static Map<String, String> get _headers => {
    "Content-Type": "application/json",
    if (_token != null) "Authorization": "Bearer $_token",
  };

  static dynamic _processResponse(http.Response response) {
    if (response.statusCode >= 200 && response.statusCode < 300) {
      return jsonDecode(response.body);
    } else {
      throw Exception("API Error ${response.statusCode}: ${response.body}");
    }
  }


  // V110: WAKE UP SERVER (Render Cold Start) — returns true if server responded
  static Future<bool> wakeUp() async {
     try {
       print("Waking up server...");
       final response = await http.get(Uri.parse(baseUrl)).timeout(const Duration(seconds: 10));
       if (response.statusCode == 200) {
         print("✅ Server awake!");
         return true;
       }
       // Server responded but with error — still "online" (just waking up)
       print("Server responded with ${response.statusCode} (still waking up)");
       return false;
     } catch (_) {
       print("Server wake-up timed out (cold start expected). Retrying in background...");
       return false;
     }
  }
  // HELPER FOR COMMANDS
  static Future<void> sendCommand(String cmd, Map<String, dynamic> payload) async {
    // Determine endpoint based on command type logic found in other files, 
    // or just assume a generic /command endpoint exists or map it.
    // Looking at usage: set_rth_alt, set_batt_threshold.
    // Server has ai_command_router (POST /command).
    // Let's assume /command or similar.
    // Actually, looking at CentralState usage, it sends "set_rth_alt".
    // Server has ai_command_router (POST /command).

    // SAFE IMPLEMENTATION:
    await post('/command', {"command": cmd, "payload": payload});
  }
}
