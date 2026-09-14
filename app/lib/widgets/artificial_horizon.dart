import 'package:flutter/material.dart';
import 'dart:math';

class ArtificialHorizon extends StatelessWidget {
  final double roll;
  final double pitch;
  final double yaw;

  const ArtificialHorizon({
    super.key, 
    required this.roll, 
    required this.pitch, 
    required this.yaw
  });

  @override
  Widget build(BuildContext context) {
    return Container(
      width: 40,
      height: 40,
      decoration: BoxDecoration(
        color: Colors.black54,
        borderRadius: BorderRadius.circular(4),
        border: Border.all(color: Colors.white24)
      ),
      child: ClipRect(
        child: CustomPaint(
          painter: HorizonPainter(roll: roll, pitch: pitch),
          child: Center(
             // Overlay a simple drone icon that rotates with YAW
             child: Transform.rotate(
               angle: yaw * pi / 180,
               child: const Icon(Icons.navigation, color: Colors.cyanAccent, size: 20)
             )
          ),
        ),
      ),
    );
  }
}

class HorizonPainter extends CustomPainter {
  final double roll;
  final double pitch;
  
  HorizonPainter({required this.roll, required this.pitch});

  @override
  void paint(Canvas canvas, Size size) {
    final center = Offset(size.width / 2, size.height / 2);
    final radius = size.width / 2;
    
    // Draw Sky/Ground
    final paintSky = Paint()..color = Colors.blue.withOpacity(0.5);
    final paintGround = Paint()..color = Colors.brown.withOpacity(0.5);
    
    canvas.save();
    canvas.translate(center.dx, center.dy);
    canvas.rotate(-roll * pi / 180); // Roll
    canvas.translate(0, pitch * 2); // Pitch (simple scaling)
    
    canvas.drawRect(Rect.fromLTWH(-100, -100, 200, 100), paintSky);
    canvas.drawRect(Rect.fromLTWH(-100, 0, 200, 100), paintGround);
    
    canvas.drawLine(const Offset(-20, 0), const Offset(20, 0), Paint()..color = Colors.white);
    
    canvas.restore();
  }

  @override
  bool shouldRepaint(covariant HorizonPainter oldDelegate) => true;
}
