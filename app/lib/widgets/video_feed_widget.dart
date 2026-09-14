import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:webview_flutter/webview_flutter.dart';
import 'package:webview_flutter_android/webview_flutter_android.dart';
import '../services/central_state.dart';

// --------------------------------------------------
// VIDEO FEED — real WebRTC via MediaMTX (same stream the browser plays).
// Replaces the old slow JPEG-snapshot polling. Smooth, low-latency, full frame.
// --------------------------------------------------
class VideoFeed extends StatefulWidget {
  const VideoFeed({super.key});
  @override
  State<VideoFeed> createState() => _VideoFeedState();
}

class _VideoFeedState extends State<VideoFeed> {
  WebViewController? _controller;
  String _url = "";

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addPostFrameCallback((_) => _init());
  }

  void _init() {
    final state = Provider.of<CentralState>(context, listen: false);
    final mode = state.config['connection_mode'];
    String ip;
    if (mode == 'local') {
      ip = (state.config['local_ip'] ?? '10.195.147.217').toString();
    } else {
      // tailscale (default) and anything else -> direct Radxa
      ip = (state.config['tailscale_ip'] ?? '100.64.0.30').toString();
    }
    // MediaMTX WebRTC player page — identical to http://<ip>:8889/gopro in the browser
    _url = "http://$ip:8889/gopro";

    final controller = WebViewController()
      ..setJavaScriptMode(JavaScriptMode.unrestricted)
      ..setBackgroundColor(Colors.black)
      ..loadRequest(Uri.parse(_url));

    // Android: allow WebRTC/video to autoplay without a user tap
    if (controller.platform is AndroidWebViewController) {
      (controller.platform as AndroidWebViewController)
          .setMediaPlaybackRequiresUserGesture(false);
    }

    if (mounted) setState(() => _controller = controller);
  }

  @override
  Widget build(BuildContext context) {
    if (_controller == null) {
      return Container(
        color: Colors.black,
        child: const Center(
          child: SizedBox(
            width: 22, height: 22,
            child: CircularProgressIndicator(strokeWidth: 2, color: Color(0xFF2DD4BF)),
          ),
        ),
      );
    }
    return Container(
      color: Colors.black,
      width: double.infinity,
      height: double.infinity,
      child: WebViewWidget(controller: _controller!),
    );
  }
}
