import 'api_service.dart';

class MediaService {
  static Future<List<Map<String, dynamic>>> getGalleryItems() async {
    // REAL API CALL
    try {
      final List data = await ApiService.get('/media');
      return data.cast<Map<String, dynamic>>();
    } catch (e) {
      return [];
    }
  }
}
