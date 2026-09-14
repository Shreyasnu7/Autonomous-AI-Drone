import 'package:flutter/material.dart';

class CamButton extends StatelessWidget {
  final IconData icon;
  final Color color;
  final VoidCallback onTap;
  const CamButton({super.key, required this.icon, this.color = Colors.white, required this.onTap});
  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onTap: onTap,
      child: Container(
        padding: const EdgeInsets.all(12),
        decoration: BoxDecoration(color: Colors.black45, shape: BoxShape.circle, border: Border.all(color: color.withOpacity(0.5))),
        child: Icon(icon, color: color, size: 24),
      ),
    );
  }
}
