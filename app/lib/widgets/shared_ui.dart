import 'dart:math';
import 'dart:ui';
import 'package:flutter/material.dart';

// --- SHARED WIDGETS ---

class GlassContainer extends StatelessWidget {
  final Widget child;
  final double borderRadius;
  final Color color;
  final Color borderColor;
  const GlassContainer({super.key, required this.child, this.borderRadius = 12, this.color = Colors.black45, this.borderColor = Colors.white10});
  @override
  Widget build(BuildContext context) {
    return ClipRRect(
      borderRadius: BorderRadius.circular(borderRadius),
      child: BackdropFilter(
        filter: ImageFilter.blur(sigmaX: 10, sigmaY: 10),
        child: Container(
          decoration: BoxDecoration(color: color, borderRadius: BorderRadius.circular(borderRadius), border: Border.all(color: borderColor)),
          child: child,
        ),
      ),
    );
  }
}

class VirtualJoystick extends StatefulWidget {
  final double size;
  final Color color;
  final Function(Offset) onMove;
  const VirtualJoystick({super.key, required this.size, required this.color, required this.onMove});
  @override
  State<VirtualJoystick> createState() => _VirtualJoystickState();
}

class _VirtualJoystickState extends State<VirtualJoystick> {
  Offset knobPos = Offset.zero;
  void _update(Offset localPos) {
    final radius = widget.size / 2;
    final center = Offset(radius, radius);
    Offset offset = localPos - center;
    if (offset.distance > radius) offset = Offset.fromDirection(offset.direction, radius);
    
    // P1.4: DEADZONE LOGIC - Prevents drift and jitter (RC-quality control)
    final normalizedDist = offset.distance / radius;
    const centerDeadzone = 0.05; // 5% center deadzone (eliminates drift)
    const edgeDeadzone = 0.02;   // 2% edge deadzone (prevents overshoot)
    
    Offset finalOffset = offset;
    if (normalizedDist < centerDeadzone) {
      // Within center deadzone - return zero (no drift!)
      finalOffset = Offset.zero;
    } else if (normalizedDist > (1.0 - edgeDeadzone)) {
      // Near edge - clamp to prevent jitter
      finalOffset = Offset.fromDirection(offset.direction, radius * (1.0 - edgeDeadzone));
    } else {
      // Normal range - scale to remove deadzone from output
      final scaledDist = (normalizedDist - centerDeadzone) / (1.0 - centerDeadzone - edgeDeadzone);
      finalOffset = Offset.fromDirection(offset.direction, radius * scaledDist);
    }
    
    setState(() => knobPos = finalOffset);
    widget.onMove(Offset(finalOffset.dx / radius, finalOffset.dy / radius));
  }
  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onPanStart: (d) => _update(d.localPosition),
      onPanUpdate: (d) => _update(d.localPosition),
      onPanEnd: (d) { setState(() => knobPos = Offset.zero); widget.onMove(Offset.zero); },
      child: Container(
        width: widget.size, height: widget.size,
        decoration: BoxDecoration(color: Colors.white10, shape: BoxShape.circle, border: Border.all(color: Colors.white12)),
        child: Center(child: Transform.translate(offset: knobPos, child: Container(width: widget.size * 0.4, height: widget.size * 0.4, decoration: BoxDecoration(color: Colors.white, shape: BoxShape.circle, boxShadow: [BoxShadow(color: widget.color.withOpacity(0.5), blurRadius: 15)])))),
      ),
    );
  }
}

// --- PAINTERS ---

class GridPainter extends CustomPainter {
  final Color color;
  GridPainter({required this.color});
  @override
  void paint(Canvas canvas, Size size) {
    final p = Paint()..color = color..strokeWidth = 1;
    for (double i = 0; i < size.width; i+=40) canvas.drawLine(Offset(i, 0), Offset(i, size.height), p);
    for (double i = 0; i < size.height; i+=40) canvas.drawLine(Offset(0, i), Offset(size.width, i), p);
  }
  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => false;
}

class PathPainter extends CustomPainter {
  final List<Offset> points;
  final Color color;
  PathPainter({required this.points, required this.color});
  @override
  void paint(Canvas canvas, Size size) {
    if (points.isEmpty) return;
    final p = Paint()..color = color..strokeWidth = 3..style = PaintingStyle.stroke..strokeCap = StrokeCap.round;
    final path = Path()..moveTo(points.first.dx, points.first.dy);
    for (var i = 1; i < points.length; i++) path.lineTo(points[i].dx, points[i].dy);
    canvas.drawPath(path, p);
    final dot = Paint()..color = Colors.white;
    for (var pt in points) canvas.drawCircle(pt, 4, dot);
  }
  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => true;
}

class True3DPainter extends CustomPainter {
  final double angle;
  final Color color;
  final double tiltX;
  True3DPainter({required this.angle, required this.color, this.tiltX = 0.4});
  @override
  void paint(Canvas canvas, Size size) {
    final cx = size.width / 2;
    final cy = size.height / 2;
    final paint = Paint()..color = color..style = PaintingStyle.stroke..strokeWidth = 2;
    final fillPaint = Paint()..color = Colors.black..style = PaintingStyle.fill;
    final List<List<double>> points = [[0, 0, 0], [-40, 0, -40], [40, 0, -40], [-40, 0, 40], [40, 0, 40], [0, -10, 0], [0, 10, 0]];
    final cosA = cos(angle);
    final sinA = sin(angle);
    final tilt = tiltX;
    final cosT = cos(tilt);
    final sinT = sin(tilt);
    List<Offset> projected = [];
    List<double> zDepth = [];
    for (var p in points) {
      double x = p[0]; double y = p[1]; double z = p[2];
      double x1 = x * cosA - z * sinA;
      double z1 = x * sinA + z * cosA;
      double y2 = y * cosT - z1 * sinT;
      double z2 = y * sinT + z1 * cosT;
      // Increased Scale for bigger model visualization
      double scale = 500 / (300 + z2 + 100); 
      projected.add(Offset(cx + x1 * scale, cy + y2 * scale));
      zDepth.add(z2);
    }
    for (int i = 1; i <= 4; i++) {
       paint.color = color.withOpacity((1 - (zDepth[i]/100)).clamp(0.3, 1.0));
       canvas.drawLine(projected[0], projected[i], paint); 
       canvas.drawCircle(projected[i], 8, fillPaint);
       canvas.drawCircle(projected[i], 8, paint);
       canvas.drawOval(Rect.fromCenter(center: projected[i].translate(0, -5), width: 50, height: 10), paint..strokeWidth=1);
    }
    canvas.drawCircle(projected[5], 15, fillPaint); 
    canvas.drawCircle(projected[5], 15, paint..strokeWidth=2..color=color);
    Offset front = Offset((projected[1].dx + projected[2].dx)/2, (projected[1].dy + projected[2].dy)/2);
    canvas.drawLine(projected[0], front, paint..color=Colors.redAccent);
  }
  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => true;
}


