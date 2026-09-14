import 'package:flutter/material.dart';
import '../theme/app_theme.dart';
import '../services/media_service.dart';
import 'package:url_launcher/url_launcher.dart';
import 'package:dio/dio.dart';
import 'package:gal/gal.dart';
import 'package:path_provider/path_provider.dart';

class GalleryScreen extends StatefulWidget {
  const GalleryScreen({super.key});

  @override
  State<GalleryScreen> createState() => _GalleryScreenState();
}

class _GalleryScreenState extends State<GalleryScreen> with SingleTickerProviderStateMixin {
  late TabController _tabController;
  List<Map<String, dynamic>> _mediaItems = [];
  bool _loading = true;

  @override
  void initState() {
    super.initState();
    _tabController = TabController(length: 2, vsync: this);
    _loadMedia();
  }

  Future<void> _loadMedia() async {
    final items = await MediaService.getGalleryItems();
    if(mounted) setState(() { _mediaItems = items; _loading = false; });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text("Media Gallery"),
        backgroundColor: Colors.transparent,
        elevation: 0,
        bottom: TabBar(
          controller: _tabController,
          indicatorColor: AppTheme.accentColor,
          tabs: const [
            Tab(text: "PHOTOS"),
            Tab(text: "VIDEOS"),
          ],
        ),
      ),
      body: Container(
        decoration: const BoxDecoration(
          image: DecorationImage(
            image: AssetImage("assets/api/grid_bg.png"), // Placeholder or just use color
            fit: BoxFit.cover,
            opacity: 0.1,
          ),
        ),
        child: TabBarView(
          controller: _tabController,
          children: [
            _buildGrid(isPhoto: true),
            _buildGrid(isPhoto: false),
          ],
        ),
      ),
    );
  }

  Widget _buildGrid({required bool isPhoto}) {
    if (_loading) return const Center(child: CircularProgressIndicator(color: AppTheme.neonBlue));
    
    // Filter items
    final type = isPhoto ? "photo" : "video";
    final items = _mediaItems.where((m) => m["type"] == type).toList();

    return GridView.builder(
      padding: const EdgeInsets.all(16),
      gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(
        crossAxisCount: 3,
        crossAxisSpacing: 10,
        mainAxisSpacing: 10,
      ),
      itemCount: items.length, 
      itemBuilder: (context, index) {
        final item = items[index];
        final url = item['url'].toString();
        
        return Container(
          decoration: BoxDecoration(
            color: Colors.white.withOpacity(0.1),
            borderRadius: BorderRadius.circular(8),
            border: Border.all(color: Colors.white.withOpacity(0.2)),
            image: url.isNotEmpty 
               ? DecorationImage(image: NetworkImage(url), fit: BoxFit.cover)
               : null
          ),
          child: Stack(
            children: [
               // Full Tap -> Open
               Positioned.fill(
                 child: Material(
                   color: Colors.transparent,
                   child: InkWell(
                     onTap: () async {
                         if (url.isNotEmpty) {
                             final uri = Uri.parse(url);
                             if (await canLaunchUrl(uri)) {
                                await launchUrl(uri, mode: LaunchMode.externalApplication);
                             }
                         }
                     },
                   ),
                 )
               ),
               
               // Download Button (Bottom Right)
               Positioned(
                 bottom: 4,
                 right: 4,
                 child: GestureDetector(
                   onTap: () => _saveToGallery(url, isPhoto),
                   child: Container(
                     padding: const EdgeInsets.all(6),
                     decoration: const BoxDecoration(
                       color: Colors.black54,
                       shape: BoxShape.circle
                     ),
                     child: const Icon(Icons.download, size: 18, color: AppTheme.neonBlue),
                   ),
                 ),
               ),
            ],
          ),
        );
      },
    );
  }



  Future<void> _saveToGallery(String url, bool isPhoto) async {
      if (url.isEmpty) return;
      
      ScaffoldMessenger.of(context).showSnackBar(const SnackBar(content: Text("Downloading..."), duration: Duration(seconds: 1)));
      
      try {
          final tempDir = await getTemporaryDirectory();
          final path = "${tempDir.path}/${DateTime.now().millisecondsSinceEpoch}.${isPhoto ? 'jpg' : 'mp4'}";
          
          await Dio().download(url, path);
          
          // Use GAL (Modern, No Conflicts)
          if (isPhoto) {
              await Gal.putImage(path, album: "DroneMedia");
          } else {
              await Gal.putVideo(path, album: "DroneMedia");
          }
           
          ScaffoldMessenger.of(context).showSnackBar(const SnackBar(content: Text("✅ Saved to Gallery!"), backgroundColor: Colors.green));

      } catch (e) {
         if (e.toString().contains("ACCESS_DENIED")) {
            // Gal handles permissions, but if denied:
            ScaffoldMessenger.of(context).showSnackBar(const SnackBar(content: Text("Permission Denied"), backgroundColor: Colors.red));
         } else {
            ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text("Save Failed: $e"), backgroundColor: Colors.red));
         }
      }
  }
}
