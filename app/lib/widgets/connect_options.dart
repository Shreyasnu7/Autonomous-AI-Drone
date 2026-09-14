import 'package:flutter/material.dart';
import '../theme/app_theme.dart';

class ConnectOptionsWidget extends StatefulWidget {
  final Function(String) onConnectTypeSelected;

  const ConnectOptionsWidget({super.key, required this.onConnectTypeSelected});

  @override
  State<ConnectOptionsWidget> createState() => _ConnectOptionsWidgetState();
}

class _ConnectOptionsWidgetState extends State<ConnectOptionsWidget> {
  String? _hovered;

  @override
  Widget build(BuildContext context) {
    return SingleChildScrollView(
      scrollDirection: Axis.horizontal,
      child: Row(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          _buildCard("CLOUD CONTROL", "Configure cloud host in settings", Icons.cloud_outlined, const Color(0xFF2DD4BF)),
          const SizedBox(width: 20),
          _buildCard("LOCAL WIFI", "Enter local drone IP", Icons.wifi, const Color(0xFFFF9E42)),
          const SizedBox(width: 20),
          _buildCard("BLUETOOTH (PAIR)", "Scan & pair (not implemented)", Icons.bluetooth, const Color(0xFF5E9EFF)),
        ],
      ),
    );
  }

  Widget _buildCard(String title, String subtitle, IconData icon, Color color) {
    final isHovered = _hovered == title;
    
    return MouseRegion(
      onEnter: (_) => setState(() => _hovered = title),
      onExit: (_) => setState(() => _hovered = null),
      child: GestureDetector(
        onTap: () => widget.onConnectTypeSelected(title.split(" ").first),
        child: AnimatedContainer(
          duration: const Duration(milliseconds: 200),
          width: 280,
          height: 140, // Increased to prevent overflow
          padding: const EdgeInsets.symmetric(horizontal: 20, vertical: 15),
          decoration: BoxDecoration(
            color: Colors.black.withOpacity(0.6),
            borderRadius: BorderRadius.circular(15),
            border: Border.all(
              color: isHovered ? color : color.withOpacity(0.3),
              width: isHovered ? 2 : 1
            ),
            boxShadow: [
              if (isHovered) BoxShadow(color: color.withOpacity(0.2), blurRadius: 20)
            ]
          ),
          child: Row(
            children: [
              Container(
                padding: const EdgeInsets.all(10),
                decoration: BoxDecoration(
                  shape: BoxShape.circle,
                  border: Border.all(color: color.withOpacity(0.5))
                ),
                child: Icon(icon, color: color, size: 24),
              ),
              const SizedBox(width: 15),
              Expanded(
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    Text(title, style: TextStyle(color: color, fontWeight: FontWeight.bold, fontSize: 14)),
                    const SizedBox(height: 5),
                    Text(subtitle, style: const TextStyle(color: Colors.grey, fontSize: 10), maxLines: 2, overflow: TextOverflow.ellipsis),
                  ],
                ),
              ),
              if (isHovered)
                Icon(Icons.arrow_forward_ios, color: color, size: 14)
            ],
          ),
        ),
      ),
    );
  }
}
