import 'api_service.dart';

class WeatherService {
  static Future<Map<String, dynamic>> getWeather(double lat, double lng) async {
    // In a real app, you might use OpenMeteo or your own backend
    try {
      final data = await ApiService.get('/weather?lat=$lat&lng=$lng');
      return data;
    } catch (_) {
      return {"temp": 0, "wind_speed": 0, "condition": "N/A"};
    }
  }
}
