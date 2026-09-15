import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../services/central_state.dart';
import '../widgets/shared_ui.dart'; // For 3D Painter if needed

class AnalyticsSidebar extends StatelessWidget {
  final VoidCallback onClose;
  // Real Data Injection
  final List<double> altHistory;
  final List<double> speedHistory;
  final List<double> batHistory;
  final double currentAlt;
  final double currentSpeed;
  final double currentDist;
  // ATTITUDE
  final double currentRoll;
  final double currentPitch;
  final double currentYaw;

  const AnalyticsSidebar({
    super.key, 
    required this.onClose,
    this.altHistory = const [],
    this.speedHistory = const [],
    this.batHistory = const [],
    this.currentAlt = 0.0,
    this.currentSpeed = 0.0,
    this.currentDist = 0.0,
    this.currentRoll = 0.0,
    this.currentPitch = 0.0,
    this.currentYaw = 0.0,
  });

  @override
  Widget build(BuildContext context) {
    return Container(
      width: 350,
      height: double.infinity,
      decoration: BoxDecoration(
        color: const Color(0xFF111111).withOpacity(0.95),
        border: const Border(left: BorderSide(color: Color(0xFF3B82F6), width: 1)), // Blue for analytics
        boxShadow: const [BoxShadow(color: Colors.black, blurRadius: 20)],
      ),
      child: Column(
        children: [
           Padding(
            padding: const EdgeInsets.all(20),
            child: Row(
              mainAxisAlignment: MainAxisAlignment.spaceBetween,
              children: [
                const Text("FLIGHT ANALYTICS", style: TextStyle(color: Colors.white, fontSize: 18, fontWeight: FontWeight.bold, letterSpacing: 2)),
                IconButton(icon: const Icon(Icons.close, color: Colors.white54), onPressed: onClose)
              ],
            ),
          ),
          const Divider(color: Colors.white10),
          Expanded(
            child: ListView(
              padding: const EdgeInsets.all(20),
              children: [
                // 3D ORIENTATION VISUALIZER (Requested by User)
                const Text("LIVE ORIENTATION", style: TextStyle(color: Colors.white54, fontSize: 10)),
                Container(
                   height: 150,
                   width: double.infinity,
                   margin: const EdgeInsets.symmetric(vertical: 10),
                   decoration: BoxDecoration(color: Colors.black, border: Border.all(color: Colors.white10)),
                   child: CustomPaint(
                      // Real Attitude Binding
                      // Note: True3DPainter takes angle (yaw) and tiltX (pitch). 
                      // For full roll support, we'd update the painter, but this removes the 'static' 0.5 value.
                      painter: True3DPainter(angle: currentYaw, tiltX: currentPitch, roll: currentRoll, color: Colors.blue),
                   ),
                ),
                
                _GraphWidget(label: "ALTITUDE", color: Colors.blue, data: altHistory, maxVal: 150),
                const SizedBox(height: 20),
                _GraphWidget(label: "SPEED", color: Colors.green, data: speedHistory, maxVal: 30),
                const SizedBox(height: 20),
                // Battery is usually 100 to 0
                _GraphWidget(label: "BATTERY (%)", color: Colors.orange, data: batHistory, maxVal: 100),
                
                const SizedBox(height: 20),
                Container(
                  padding: const EdgeInsets.all(16),
                  decoration: BoxDecoration(color: Colors.white.withOpacity(0.05), borderRadius: BorderRadius.circular(10)),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      const Text("SESSION SUMMARY", style: TextStyle(color: Colors.white54, fontSize: 12)),
                      const SizedBox(height: 10),
                      _StatRow(label: "Cur Altitude", val: context.read<CentralState>().formatDist(currentAlt)),
                      _StatRow(label: "Cur Speed", val: context.read<CentralState>().formatSpeed(currentSpeed)),
                      _StatRow(label: "Distance", val: context.read<CentralState>().formatDist(currentDist)),
                    ],
                  ),
                )
              ],
            ),
          )
        ],
      ),
    );
  }
}

class _GraphWidget extends StatelessWidget {
  final String label;
  final Color color;
  final List<double> data;
  final double maxVal;
  
  const _GraphWidget({required this.label, required this.color, required this.data, required this.maxVal});

  @override
  Widget build(BuildContext context) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Row(mainAxisAlignment: MainAxisAlignment.spaceBetween, children: [
           Text(label, style: const TextStyle(color: Colors.white54, fontSize: 10)),
           Text(data.isNotEmpty ? data.last.toStringAsFixed(1) : "-", style: TextStyle(color: color, fontWeight: FontWeight.bold, fontSize: 12)),
        ]),
        const SizedBox(height: 5),
        Container(
          height: 100,
          width: double.infinity,
          padding: const EdgeInsets.all(10),
          decoration: BoxDecoration(border: Border.all(color: Colors.white10), borderRadius: BorderRadius.circular(5)),
          child: CustomPaint(painter: _RealGraphPainter(color: color, data: data, maxVal: maxVal)),
        )
      ],
    );
  }
}

class _RealGraphPainter extends CustomPainter {
  final Color color;
  final List<double> data;
  final double maxVal;
  
  _RealGraphPainter({required this.color, required this.data, required this.maxVal});
  
  @override
  void paint(Canvas canvas, Size size) {
    final paint = Paint()..color = color..style = PaintingStyle.stroke..strokeWidth = 2;
    if (data.isEmpty) return;
    
    final path = Path();
    final stepX = size.width / (data.length > 1 ? data.length - 1 : 1);
    
    // Normalize and plot
    for (int i = 0; i < data.length; i++) {
      final x = i * stepX;
      // Invert Y because canvas 0 is top. 
      // y = height - (val / maxVal * height)
      final val = data[i].clamp(0.0, maxVal);
      final y = size.height - (val / maxVal * size.height);
      
      if (i == 0) path.moveTo(x, y);
      else path.lineTo(x, y);
    }
    
    canvas.drawPath(path, paint);
    
    // Fill
    final fillPaint = Paint()..color = color.withOpacity(0.1)..style = PaintingStyle.fill;
    path.lineTo(size.width, size.height);
    path.lineTo(0, size.height);
    path.close();
    canvas.drawPath(path, fillPaint);
  }
  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => true;
}

class _StatRow extends StatelessWidget {
  final String label, val;
  const _StatRow({required this.label, required this.val});
  @override
  Widget build(BuildContext context) {
    return Padding(padding: const EdgeInsets.only(bottom: 5), child: Row(mainAxisAlignment: MainAxisAlignment.spaceBetween, children: [Text(label, style: const TextStyle(color: Colors.white)), Text(val, style: const TextStyle(color: Colors.white, fontWeight: FontWeight.bold))]));
  }
}
