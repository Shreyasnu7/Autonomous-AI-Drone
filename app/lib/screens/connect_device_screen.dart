import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../widgets/shared_ui.dart';
import '../services/central_state.dart';
import '../widgets/connect_options.dart';
import 'package:animate_do/animate_do.dart';
import '../services/api_service.dart';

class ConnectScreen extends StatefulWidget {
  final bool isAddingMore; 
  const ConnectScreen({super.key, this.isAddingMore = false});

  @override
  State<ConnectScreen> createState() => _ConnectScreenState();
}

class _ConnectScreenState extends State<ConnectScreen> with SingleTickerProviderStateMixin {
  final _deviceCtrl = TextEditingController();
  bool _searching = true;
  late AnimationController _pulse;

  @override
  void initState() {
    super.initState();
    _pulse = AnimationController(vsync: this, duration: const Duration(seconds: 2))..repeat();
    // REAL SEARCH LOGIC: Check Cloud Server for paired drones
    _searchForDrones();
  }

  Future<void> _searchForDrones() async {
     try {
       // Attempt to fetch available drones via API to see if anything is online
       // We use a short timeout so we don't block the UI forever
       await ApiService.get('/drones').timeout(const Duration(seconds: 30));
       
       // If successful (or even if empty list returned), we stop "Searching" animation
       if (mounted) setState(() => _searching = false);
     } catch (e) {
       // If API fails (offline), we also stop searching and show manual connect
       if (mounted) setState(() => _searching = false);
     }
  }

  @override
  void dispose() {
    _pulse.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final neonTeal = const Color(0xFF2DD4BF);
    final neonOrange = const Color(0xFFFF9E42);
    
    return Scaffold(
      backgroundColor: Colors.black,
      body: Stack(
        fit: StackFit.expand,
        children: [
          // Background Grid
          Positioned.fill(
            child: CustomPaint(painter: GridPainter(color: Colors.white.withOpacity(0.03))),
          ),
          
          // Vignette
          Container(
            decoration: BoxDecoration(
              gradient: RadialGradient(
                colors: [Colors.transparent, Colors.black.withOpacity(0.8)],
                radius: 1.2,
              )
            ),
          ),

          Center(
            child: SingleChildScrollView(
              padding: const EdgeInsets.symmetric(vertical: 40),
              child: FadeInUp(
                duration: const Duration(milliseconds: 800),
                child: Column(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                   // Signal Pulse
                   SizedBox(
                      height: 180,
                      child: Stack(
                        alignment: Alignment.center,
                        children: [
                          ScaleTransition(scale: Tween(begin: 1.0, end: 1.5).animate(_pulse), child: FadeTransition(opacity: Tween(begin: 0.3, end: 0.0).animate(_pulse), child: Container(decoration: BoxDecoration(shape: BoxShape.circle, border: Border.all(color: neonTeal, width: 2))))),
                          Container(
                            padding: const EdgeInsets.all(20),
                            decoration: BoxDecoration(shape: BoxShape.circle, color: Colors.black, border: Border.all(color: neonTeal, width: 2), boxShadow: [BoxShadow(color: neonTeal.withOpacity(0.3), blurRadius: 20)]),
                            child: Icon(Icons.wifi_tethering, size: 40, color: neonTeal),
                          )
                        ],
                      ),
                   ),
                   const SizedBox(height: 10),
                   const Text("NO SIGNAL", style: TextStyle(color: Color(0xFF2DD4BF), fontSize: 16, letterSpacing: 3, fontWeight: FontWeight.bold, fontFamily: 'Courier')),
                   const SizedBox(height: 10),
                   Text("No drone found (use Cloud, Local or Pair manually)", style: TextStyle(color: Colors.white.withOpacity(0.5), fontSize: 12, fontFamily: 'Courier')),
                   
                   const SizedBox(height: 5),
                   SizedBox(
                     width: 200,
                     child: Column(children: [
                        const Divider(color: Colors.white10),
                        Text("Signal: 0%", style: TextStyle(color: Colors.white.withOpacity(0.3), fontSize: 10, fontFamily: 'Courier')),
                        const SizedBox(height: 5),
                        Text("Latency: -- ms", style: TextStyle(color: Colors.white.withOpacity(0.3), fontSize: 10, fontFamily: 'Courier')),
                        const Divider(color: Colors.white10),
                     ]),
                   ),

                   const SizedBox(height: 50),
                   
                   // Connection Cards
                   ConnectOptionsWidget(onConnectTypeSelected: (type) async {
                        // USE USER TYPED NAME IF AVAILABLE
                        String name = _deviceCtrl.text.trim();
                        if (name.isEmpty) name = "Neon-Drone-$type";

                        // Set connection mode from the picked card (TAILSCALE/CLOUD/LOCAL/BLUETOOTH)
                        context.read<CentralState>().updateConfig('connection_mode', type.toLowerCase());

                        await context.read<CentralState>().connectDevice(name);
                        if (mounted) Navigator.of(context).pushNamedAndRemoveUntil('/hangar', (route) => false);
                   }),

                   const SizedBox(height: 50),

                   // Register Device Form
                   Container(
                     width: 500,
                     padding: const EdgeInsets.all(30),
                     decoration: BoxDecoration(
                       color: Colors.black,
                       borderRadius: BorderRadius.circular(10),
                       border: Border.all(color: Colors.white12)
                     ),
                     child: Column(
                       children: [
                         const Text("REGISTER DEVICE", style: TextStyle(color: Colors.white, fontFamily: 'Courier', letterSpacing: 2, fontWeight: FontWeight.bold)),
                         const SizedBox(height: 20),
                         
                         TextField(
                           controller: _deviceCtrl,
                           style: const TextStyle(color: Colors.white, fontFamily: 'Courier'),
                           decoration: InputDecoration(
                             prefixIcon: Icon(Icons.connecting_airports, color: neonTeal),
                             hintText: "DEVICE NAME",
                             hintStyle: TextStyle(color: neonTeal.withOpacity(0.5), fontFamily: 'Courier'),
                             filled: true,
                             fillColor: Colors.transparent,
                             enabledBorder: OutlineInputBorder(borderSide: const BorderSide(color: Colors.white24)),
                             focusedBorder: OutlineInputBorder(borderSide: BorderSide(color: neonTeal))
                           ),
                         ),
                         
                         const SizedBox(height: 20),
                         
                         Row(
                           children: [
                             Expanded(
                               child: ElevatedButton.icon(
                                 style: ElevatedButton.styleFrom(backgroundColor: neonOrange, foregroundColor: Colors.black, padding: const EdgeInsets.symmetric(vertical: 15)),
                                 onPressed: _showLocalIpDialog, 
                                 icon: const Icon(Icons.wifi), 
                                 label: const Text("PAIR LOCAL")
                               )
                             ),
                             const SizedBox(width: 20),
                             Expanded(
                               child: ElevatedButton.icon(
                                 style: ElevatedButton.styleFrom(backgroundColor: neonTeal, foregroundColor: Colors.black, padding: const EdgeInsets.symmetric(vertical: 15)),
                                 onPressed: () => _addLog("Use Cloud Connect above"), 
                                 icon: const Icon(Icons.cloud_upload), 
                                 label: const Text("PAIR CLOUD")
                               )
                             ),
                           ],
                         ),
                         const SizedBox(height: 20),
                         Text("SAVE & CONTINUE", style: TextStyle(color: Colors.white.withOpacity(0.3), fontSize: 10, letterSpacing: 2)),
                       ],
                     ),
                   )
                ],
              ),
            ),
          ),
          ),
        ],
      ),
    );
  }

  void _addLog(String msg) {
     ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text(msg), backgroundColor: Colors.grey[900]));
  }

  void _showLocalIpDialog() {
     final ipCtrl = TextEditingController(text: "192.168.100.10"); // Default Radxa AP IP
     showDialog(context: context, builder: (c) => AlertDialog(
        backgroundColor: Colors.grey[900],
        title: const Text("Local Connection", style: TextStyle(color: Colors.white)),
        content: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            const Text("Enter Drone IP Address", style: TextStyle(color: Colors.white70)),
            const SizedBox(height: 10),
            TextField(controller: ipCtrl, style: const TextStyle(color: Colors.white), decoration: const InputDecoration(filled: true, fillColor: Colors.black, border: OutlineInputBorder(), hintText: "e.g. 192.168.1.10"))
          ],
        ),
        actions: [
           TextButton(onPressed: () => Navigator.pop(c), child: const Text("CANCEL")),
           ElevatedButton(
             style: ElevatedButton.styleFrom(backgroundColor: const Color(0xFFFF9E42)),
             onPressed: () {
                Navigator.pop(c);
                _connectLocal(ipCtrl.text.trim());
             },
             child: const Text("CONNECT", style: TextStyle(color: Colors.black))
           )
        ],
     ));
  }

  Future<void> _connectLocal(String ip) async {
      if (ip.isEmpty) return;
      _addLog("Connecting to $ip...");
      // Direct WebSocket Connection Logic
      await context.read<CentralState>().connectDevice(ip); // This handles WS connection if IP is provided
      if (mounted) Navigator.of(context).pushNamedAndRemoveUntil('/hangar', (route) => false);
  }
}
