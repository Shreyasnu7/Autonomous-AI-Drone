import 'package:flutter/material.dart';

class ReadinessRow extends StatelessWidget {
  final String label, value;
  final IconData icon;
  final Color color;
  const ReadinessRow({super.key, required this.label, required this.value, required this.icon, required this.color});
  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.only(bottom: 12.0),
      child: Row(
        children: [
          Icon(icon, color: color, size: 16),
          const SizedBox(width: 10),
          Text(label, style: const TextStyle(color: Colors.white70)),
          const Spacer(),
          Text(value, style: TextStyle(fontWeight: FontWeight.bold, color: color)),
        ],
      ),
    );
  }
}
