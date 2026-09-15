import 'dart:convert';
import 'package:http/http.dart' as http;
import 'api_service.dart';
import 'central_state.dart';

class MediaService {
  /// Media is recorded ON THE AIRCRAFT. Asking the cloud relay for it meant the
  /// gallery never showed anything the drone actually captured -- those files sat
  /// on the companion computer, reachable only over SSH. The bridge now serves
  /// them from its local video server, so fetch them over the same direct link
  /// the video feed uses, and fall back to the relay only if that fails.
  static String _bridgeBase(CentralState state) {
    final mode = state.config['connection_mode'];
    final ip = (mode == 'local')
        ? (state.config['local_ip'] ?? '10.195.147.217').toString()
        : (state.config['tailscale_ip'] ?? '100.64.0.30').toString();
    return "http://$ip:8080";
  }

  static Future<List<Map<String, dynamic>>> getGalleryItems({CentralState? state}) async {
    if (state != null) {
      try {
        final res = await http
            .get(Uri.parse("${_bridgeBase(state)}/media"))
            .timeout(const Duration(seconds: 8));
        if (res.statusCode == 200) {
          final List data = jsonDecode(res.body);
          return data.cast<Map<String, dynamic>>();
        }
      } catch (_) {
        // fall through to the relay
      }
    }
    try {
      final List data = await ApiService.get('/media');
      return data.cast<Map<String, dynamic>>();
    } catch (e) {
      return [];
    }
  }
}
