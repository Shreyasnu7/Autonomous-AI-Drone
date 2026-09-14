import 'dart:math';
import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:google_maps_flutter/google_maps_flutter.dart';
import 'dart:io';
import '../widgets/shared_ui.dart';

class LogDetailDialog extends StatefulWidget {
  final String date, loc, dur, status, filePath;
  const LogDetailDialog({super.key, required this.date, required this.loc, required this.dur, required this.status, required this.filePath});

  @override
  State<LogDetailDialog> createState() => _LogDetailDialogState();
}

class _LogDetailDialogState extends State<LogDetailDialog> with SingleTickerProviderStateMixin {
  late TabController _tabController;
  
  List<Offset> _flightPath = [];
  List<LatLng> _realPath = [];
  List<double> _altProfile = [];

  GoogleMapController? _mapController;
  bool _isLoading = true;

  @override
  void initState() {
    super.initState();
    _tabController = TabController(length: 3, vsync: this);
    _loadLogData();
  }

  Future<void> _loadLogData() async {
    try {
      if (widget.filePath == "path_placeholder") {
         // Fallback for mock if needed, or just return empty
         setState(() => _isLoading = false);
         return;
      }

      final file = File(widget.filePath);
      if (!await file.exists()) return;

      final lines = await file.readAsLines();
      List<LatLng> points = [];
      List<double> alts = [];

      for (var line in lines) {
        if (line.trim().isEmpty) continue;
        try {
           final data = jsonDecode(line); // Will need import dart:convert
           // {t, lat, lng, alt, spd, bat}
           final lat = (data['lat'] as num).toDouble();
           final lng = (data['lng'] as num).toDouble();
           final alt = (data['alt'] as num).toDouble();
           
           points.add(LatLng(lat, lng));
           alts.add(alt);
        } catch (_) {}
      }

      if (points.isNotEmpty) {
        setState(() {
          _realPath = points;
          _altProfile = alts;
          
          // Generate simple offsets for fallback painter (normalize to 0..1)
          // Simplified for now
          _flightPath = points.map((p) => Offset(p.latitude, p.longitude)).toList(); // Raw for now, painter handles normalization usually or expects 0.1
          _isLoading = false;
        });
      }
    } catch (e) {
      print("Error parse log: $e");
    }
  }
  
  @override
  void dispose() {
    _tabController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final size = MediaQuery.of(context).size;
    final isMobile = size.width < 600;
    
    return Dialog(
      backgroundColor: Colors.transparent,
      insetPadding: const EdgeInsets.symmetric(horizontal: 5, vertical: 5), // Minimal Inset
      child: LayoutBuilder(
        builder: (context, constraints) {
          double w = constraints.maxWidth;
          double h = constraints.maxHeight;
          
          double finalW = isMobile ? w * 0.99 : min(w * 0.98, 1200.0);
          double finalH = h * 0.99; 

          // Ensure it's big enough
          if (finalW < 600 && w > 600) finalW = 600;
          if (finalH < 600 && h > 600) finalH = 600;

          // Force at least 500px unless screen is smaller
          if (finalW < 500 && w > 500) finalW = 500;
          if (finalH < 500 && h > 500) finalH = 500;

          return Container(
            width: finalW,
            height: finalH,
            decoration: BoxDecoration(
              color: const Color(0xFF111111),
              borderRadius: BorderRadius.circular(15), 
              border: Border.all(color: const Color(0xFF2DD4BF).withOpacity(0.5)),
              boxShadow: [BoxShadow(color: const Color(0xFF2DD4BF).withOpacity(0.1), blurRadius: 20)],
            ),
            child: Column(
              children: [
                // Header 
                Container(
                  padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 5),
                  decoration: const BoxDecoration(border: Border(bottom: BorderSide(color: Colors.white10))),
                  child: Row(
                    mainAxisAlignment: MainAxisAlignment.spaceBetween,
                    children: [
                      Expanded(
                        child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
                          FittedBox(child: Text(widget.loc.toUpperCase(), style: const TextStyle(color: Colors.white, fontSize: 16, fontWeight: FontWeight.bold, letterSpacing: 1.5))),
                          Text(widget.date, style: const TextStyle(color: Colors.white54, fontSize: 10)),
                        ]),
                      ),
                      const SizedBox(width: 10),
                      Container(
                        padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 2),
                        decoration: BoxDecoration(
                          color: widget.status == "SUCCESS" ? Colors.green.withOpacity(0.2) : Colors.amber.withOpacity(0.2),
                          borderRadius: BorderRadius.circular(20),
                          border: Border.all(color: widget.status == "SUCCESS" ? Colors.green : Colors.amber),
                        ),
                        child: Text(widget.status, style: TextStyle(color: widget.status == "SUCCESS" ? Colors.green : Colors.amber, fontWeight: FontWeight.bold, fontSize: 10)),
                      )
                    ],
                  ),
                ),
                
                // Tabs
                SizedBox(
                  height: 35, 
                  child: TabBar(
                    controller: _tabController,
                    indicatorColor: const Color(0xFF2DD4BF),
                    labelColor: const Color(0xFF2DD4BF),
                    unselectedLabelColor: Colors.white54,
                    labelStyle: const TextStyle(fontSize: 10, fontWeight: FontWeight.bold),
                    tabs: const [
                      Tab(text: "OVERVIEW"),
                      Tab(text: "FLIGHT PATH"),
                      Tab(text: "GRAPHS"), 
                    ],
                  ),
                ),
                
                // Content
                Expanded(
                  child: TabBarView(
                    controller: _tabController,
                    physics: const NeverScrollableScrollPhysics(), 
                    children: [
                      // 1. OVERVIEW
                      SingleChildScrollView(
                        child: Padding(
                          padding: const EdgeInsets.all(20),
                          child: Wrap(
                            spacing: 20,
                            runSpacing: 20, 
                            alignment: WrapAlignment.center,
                            children: [
                              _DetailItem(label: "DURATION", value: widget.dur),
                              _DetailItem(label: "MAX ALT", value: "118m"),
                              _DetailItem(label: "MAX SPEED", value: "14m/s"),
                              _DetailItem(label: "BATTERY USED", value: "48%"),
                              _DetailItem(label: "PHOTOS", value: "124"),
                              _DetailItem(label: "VIDEOS", value: "4"),
                            ],
                          ),
                        ),
                      ),
                      
                      // 2. REAL MAP PATH
                      Column( 
                        children: [
                          Expanded(
                            child: Container(
                              width: double.infinity,
                              margin: const EdgeInsets.all(10),
                              decoration: BoxDecoration(color: Colors.black, border: Border.all(color: Colors.white10)),
                              child: ClipRRect(
                                 borderRadius: BorderRadius.circular(4),
                                 child: Builder(
                                   builder: (context) {
                                      bool showMap = false;
                                      try { showMap = !Platform.isLinux && !Platform.isWindows && !Platform.isMacOS; } catch (e) { showMap = true; }

                                      if (!showMap) {
                                        return Stack(
                                          children: [
                                             SizedBox.expand(child: ColoredBox(color: Colors.black, child: CustomPaint(painter: GridPainter(color: Colors.white12)))),
                                             Center(child: CustomPaint(painter: PathPainter(points: _flightPath, color: Colors.orangeAccent))),
                                             const Center(child: Text("MAP VIEW NOT SUPPORTED ON DESKTOP", style: TextStyle(color: Colors.white30, fontWeight: FontWeight.bold))),
                                          ],
                                        );
                                      }

                                      return GoogleMap(
                                        initialCameraPosition: CameraPosition(target: _realPath.first, zoom: 16),
                                        mapType: MapType.hybrid,
                                        onMapCreated: (c) => _mapController = c,
                                        polylines: {
                                          Polyline(polylineId: const PolylineId("flight_path"), points: _realPath, color: const Color(0xFF2DD4BF), width: 4)
                                        },
                                        markers: {
                                          Marker(markerId: const MarkerId("start"), position: _realPath.first, icon: BitmapDescriptor.defaultMarkerWithHue(BitmapDescriptor.hueGreen)),
                                          Marker(markerId: const MarkerId("end"), position: _realPath.last, icon: BitmapDescriptor.defaultMarkerWithHue(BitmapDescriptor.hueRed)),
                                        },
                                      );
                                   }
                                 ),
                              ),
                            ),
                          ),
                        ],
                      ),
                      
                      // 3. GRAPHS
                      Column( // Wrap in Column/Expanded to ensure fill
                        children: [
                          Expanded(
                            child: Padding(
                              padding: const EdgeInsets.symmetric(horizontal: 20, vertical: 5),
                              child: Column(
                                children: [
                                   const Text("ALTITUDE PROFILE", style: TextStyle(color: Colors.white54, fontSize: 10)),
                                   Expanded(child: Container( // Flex 1
                                     width: double.infinity,
                                     margin: const EdgeInsets.symmetric(vertical: 5),
                                     padding: const EdgeInsets.all(10),
                                     decoration: BoxDecoration(border: Border.all(color: Colors.white10), color: Colors.black54),
                                     child: CustomPaint(painter: _LogGraphPainter(data: _altProfile, color: Colors.blueAccent)),
                                   )),
                                   const SizedBox(height: 5),
                                   const Text("SPEED PROFILE", style: TextStyle(color: Colors.white54, fontSize: 10)),
                                   Expanded(child: Container( // Flex 1
                                     width: double.infinity,
                                     margin: const EdgeInsets.symmetric(vertical: 5),
                                     padding: const EdgeInsets.all(10),
                                     decoration: BoxDecoration(border: Border.all(color: Colors.white10), color: Colors.black54),
                                     child: CustomPaint(painter: _LogGraphPainter(data: _altProfile.reversed.toList(), color: Colors.greenAccent)), 
                                   )),
                                ],
                              ),
                            ),
                          ),
                        ],
                      ),
                    ],
                  ),
                ),
                
                // Footer
                Padding(
                  padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 5), // Minimized Padding
                  child: Row(
                    mainAxisAlignment: MainAxisAlignment.end,
                    children: [
                      TextButton(onPressed: () => Navigator.pop(context), child: const Text("CLOSE", style: TextStyle(color: Colors.white54))),
                      const SizedBox(width: 10),
                      ElevatedButton.icon(
                        onPressed: () {}, 
                        icon: const Icon(Icons.download), 
                        label: const Text("EXPORT LOG"),
                        style: ElevatedButton.styleFrom(backgroundColor: const Color(0xFF2DD4BF), foregroundColor: Colors.black),
                      )
                    ],
                  ),
                )
              ],
            ), 
          ); 
        }
      ),
    );
  }
}

class _DetailItem extends StatelessWidget {
  final String label, value;
  const _DetailItem({required this.label, required this.value});
  @override
  Widget build(BuildContext context) {
    return Container(
      width: 100,
      padding: const EdgeInsets.all(10),
      decoration: BoxDecoration(color: Colors.white10, borderRadius: BorderRadius.circular(10)),
      child: Column(
        mainAxisSize: MainAxisSize.min,
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          Text(label, style: const TextStyle(color: Colors.white38, fontSize: 10, letterSpacing: 1), textAlign: TextAlign.center),
          const SizedBox(height: 4),
          FittedBox(fit: BoxFit.scaleDown, child: Text(value, style: const TextStyle(color: Colors.white, fontSize: 18, fontWeight: FontWeight.bold))),
        ],
      ),
    );
  }
}

class _LogGraphPainter extends CustomPainter {
  final List<double> data;
  final Color color;
  _LogGraphPainter({required this.data, required this.color});
  @override
  void paint(Canvas canvas, Size size) {
    final paint = Paint()..color = color..style = PaintingStyle.stroke..strokeWidth = 2;
    final fillPaint = Paint()..color = color.withOpacity(0.1)..style = PaintingStyle.fill;
    
    if (data.isEmpty) return;
    
    final path = Path();
    final stepX = size.width / (data.length - 1);
    double minVal = data.reduce(min);
    double maxVal = data.reduce(max);
    if (maxVal == minVal) maxVal += 1.0;
    final range = maxVal - minVal;
    
    path.moveTo(0, size.height - ((data[0] - minVal) / range * size.height));
    
    for (int i = 1; i < data.length; i++) {
        final x = i * stepX;
        final normalized = (data[i] - minVal) / range;
        final y = size.height - (normalized * size.height);
        path.lineTo(x, y);
    }
    
    canvas.drawPath(path, paint);
    path.lineTo(size.width, size.height);
    path.lineTo(0, size.height);
    path.close();
    canvas.drawPath(path, fillPaint);
  }
  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => false;
}
