import 'api_service.dart';
import 'package:shared_preferences/shared_preferences.dart';

class AuthService {
  static Future<Map<String, dynamic>> login(String email, String password) async {
    // REAL ENDPOINT (server route is /session/create, not /auth/...)
    const endpoint = '/session/create';
    
    final data = await ApiService.post(endpoint, {
      "email": email,
      "password": password
    });
    
    // Backend likely returns 'access_token' or 'token'
    final token = data['access_token'] ?? data['token'];
    if (token != null) {
        await ApiService.saveToken(token);
    }
    
    // 🛡️ LOCAL CACHE: Save User Details so App remembers them
    if (data['user'] != null) {
        final prefs = await SharedPreferences.getInstance();
        await prefs.setString('cached_username', data['user']['username'] ?? "Pilot");
        await prefs.setString('cached_email', data['user']['email'] ?? "");
    }
    
    return data['user']; // Must exist if login success
  }

  static Future<Map<String, dynamic>> signup(String username, String email, String password) async {
    const endpoint = '/session/register';
    
    final data = await ApiService.post(endpoint, {
      "username": username,
      "email": email,
      "password": password
    });
    
    final token = data['access_token'] ?? data['token'];
    if (token != null) {
       await ApiService.saveToken(token);
    }
    return data['user'] ?? {};
  }
  
  static Future<void> logout() async {
    await ApiService.clearToken();
  }
}
