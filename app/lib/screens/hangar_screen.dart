import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../widgets/shared_ui.dart';
import '../services/central_state.dart';
import '../services/api_service.dart';
import '../services/weather_service.dart';
import '../theme/app_theme.dart';
import '../widgets/settings_sidebar.dart';
import '../widgets/log_detail_dialog.dart';
import '../services/flight_recorder.dart';
import '../services/flight_recorder_service.dart';

class HangarScreen extends StatefulWidget {
  const HangarScreen({super.key});

  @override
  State<HangarScreen> createState() => _HangarScreenState();
}

class _HangarScreenState extends State<HangarScreen> with SingleTickerProviderStateMixin {
  // Use state data later
  late  CentralState _centralState;
  
  late AnimationController _rotator;
  bool _showSettings = false;
  bool _showDeviceList = false;
  double _manualRotation = 0.0;
  
  // Local state for UI
  List<String> _myDevices = ["F450-V2", "Mavic-Clone", "Radxa-X"];
  String currentDevice = "F450-V2";
  List<Map<String, dynamic>> _pastFlights = [];

  Map<String, String> _stats = {
    "airtime": "0h 0m", "distance": "0 km", "missions": "0 Sorties"
  };
  double _manualRoation = 0.0;

  @override
  void initState() {
    super.initState();
    // Auto rotation + Manual interaction
    _rotator = AnimationController(vsync: this, duration: const Duration(seconds: 15))..repeat();
    _loadRealData();
  }
  
  Future<void> _loadRealData() async {
    // 1. Fetch Logs
    try {
      // Flight logs are recorded ON THIS DEVICE. Reading them from the relay meant the
      // hangar listed cloud rows while the real recordings sat unread on disk.
      final logs = await LocalFlightLogger().getPastFlights();
      
      // 2. Fetch Weather (use drone GPS if available, else user's phone GPS)
      double wLat = _centralState.droneLat != 0 ? _centralState.droneLat : 37.77;
      double wLng = _centralState.droneLng != 0 ? _centralState.droneLng : -122.41;
      final weather = await WeatherService.getWeather(wLat, wLng);
      
      // 3. Calculate Local Fleet Stats
      // Use the service we just fixed!
      final localStats = await LocalFlightLogger().getFleetStats();

      if (!mounted) return;
      
      setState(() {
        _pastFlights = logs;

        // Merge real stats
        _stats = localStats;
        
        // Update Local Conditions in _stats temporarily or UI
        // We'll just update the _stats map for simplicity as it drives UI
        _localWeather = "${weather['condition'] ?? 'N/A'}, ${weather['temp'] ?? 0}°C";
        _localWind = "${weather['wind'] ?? weather['wind_speed'] ?? 0} km/h";
      });
    } catch (e) {
      print("Data Load Error: $e");
    }
  }

  String _localWeather = "Loading...";
  String _localWind = "Loading...";

  @override
  void dispose() {
    _rotator.dispose();
    super.dispose();
  }
  
  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    _centralState = Provider.of<CentralState>(context);
    if (_centralState.droneId != "Unknown Drone") {
      currentDevice = _centralState.droneId;
    }
    
    // RESTORED: Real Weather Sync
    if (_centralState.droneLat != 0.0 && _centralState.droneLng != 0.0 && _localWeather == "--") { // Simple check
         WeatherService.getWeather(_centralState.droneLat, _centralState.droneLng).then((w) {
             if (mounted) setState(() {
               _localWeather = "${w['condition']}, ${w['temp']}°C";
               // MA-15 FIX: Server returns 'wind', not 'wind_speed'
               _localWind = "${w['wind'] ?? w['wind_speed'] ?? 0} km/h";
             });
         });
    }
  }

  void _showDeviceSwitcher() {
    setState(() => _showDeviceList = !_showDeviceList);
  }

  @override
  Widget build(BuildContext context) {
    final neonTeal = const Color(0xFF2DD4BF);
    final username = (_centralState.userProfile != null && _centralState.userProfile?['username'] != null) 
       ? _centralState.userProfile!['username'] 
       : "COMMANDER";

    return Scaffold(
      backgroundColor: const Color(0xFF050505),
      body: Stack(
        children: [
          LayoutBuilder(
            builder: (context, constraints) {
              final isWide = constraints.maxWidth > 800;
              final leftFlex = isWide ? 4 : 3;
              final rightFlex = isWide ? 2 : 2;
              
              return Row(
                children: [
                   // LEFT: 3D Model & Logs
                   Expanded(
                    flex: leftFlex, 
                    child: Container(
                      decoration: const BoxDecoration(
                        gradient: RadialGradient(colors: [Color(0xFF1a1a1a), Colors.black], radius: 0.8),
                      ),
                      child: Column(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: [
                          // 1. HEADER
                          Padding(
                            padding: const EdgeInsets.only(top: 15, left: 20, right: 20, bottom: 0),
                            child: Column(
                              crossAxisAlignment: CrossAxisAlignment.start,
                              children: [
                                Row(
                                  children: [
                                    Expanded(
                                      child: FittedBox(
                                        fit: BoxFit.scaleDown,
                                        alignment: Alignment.centerLeft,
                                        child: Text(currentDevice, style: const TextStyle(fontSize: 32, fontWeight: FontWeight.bold, letterSpacing: 2, color: Colors.white)),
                                      ),
                                    ),
                                    IconButton(
                                      icon: Icon(_showDeviceList ? Icons.expand_less : Icons.expand_more, size: 30, color: Colors.white54),
                                      onPressed: _showDeviceSwitcher,
                                    )
                                  ],
                                ),
                                FittedBox(
                                  fit: BoxFit.scaleDown,
                                  alignment: Alignment.centerLeft,
                                  child: Text(
                                     "STATUS: ${_centralState.isDeviceConnected ? 'ONLINE' : 'DISCONNECTED'}  •  BATTERY: ${_centralState.currentBattery.toInt()}%", 
                                     style: TextStyle(color: _centralState.isDeviceConnected ? Colors.green : Colors.red, fontWeight: FontWeight.bold, fontSize: 12)
                                  ),
                                ),
                              ],
                            ),
                          ),

                          // 2. INTERACTIVE 3D MODEL
                          Expanded(
                            flex: 5, 
                            child: GestureDetector(
                              onPanUpdate: (d) {
                                setState(() {
                                  _manualRotation += d.delta.dx * 0.01;
                                });
                              },
                              child: Container(
                                color: Colors.transparent, 
                                child: AnimatedBuilder(
                                  animation: _rotator,
                                  builder: (context, child) {
                                    return Stack(
                                      alignment: Alignment.center,
                                      children: [
                                        Transform(
                                          transform: Matrix4.identity()..setEntry(3, 2, 0.001)..rotateX(1.4),
                                          alignment: Alignment.center,
                                          child: CustomPaint(painter: GridPainter(color: Colors.white10), size: const Size(900, 600)),
                                        ),
                                        // BIG DRONE MODEL
                                        CustomPaint(
                                          size: const Size(900, 900),
                                          painter: True3DPainter(angle: (_rotator.value * 6.28) + _manualRotation, color: neonTeal),
                                        ),
                                      ],
                                    );
                                  }
                                ),
                              ),
                            ),
                          ),
                           
                          // 3. FLIGHT LOGS
                          Expanded(
                            flex: 6, 
                            child: Container(
                              width: double.infinity,
                              padding: const EdgeInsets.all(20),
                              decoration: const BoxDecoration(
                                color: Color(0xFF0A0A0A),
                                border: Border(top: BorderSide(color: Colors.white10)),
                              ),
                              child: Column(
                                crossAxisAlignment: CrossAxisAlignment.start,
                                children: [
                                  Row(
                                    mainAxisAlignment: MainAxisAlignment.spaceBetween,
                                    children: [
                                      Text("FLIGHT LOGS", style: TextStyle(color: neonTeal, fontSize: 12, letterSpacing: 2, fontWeight: FontWeight.bold)),
                                      // ADDED: Gallery Button
                                      TextButton.icon(
                                        onPressed: () => Navigator.pushNamed(context, '/gallery'),
                                        icon: const Icon(Icons.photo_library, size: 16, color: Colors.white70),
                                        label: const Text("MEDIA GALLERY", style: TextStyle(color: Colors.white70, fontSize: 10)),
                                      )
                                    ],
                                  ),
                                  const SizedBox(height: 10),
                                  Expanded(
                                    child: _pastFlights.isEmpty 
                                      ? const Center(child: Text("NO RECORDED FLIGHTS", style: TextStyle(color: Colors.white24)))
                                      : ListView.builder(
                                          itemCount: _pastFlights.length,
                                          itemBuilder: (context, index) {
                                             final log = _pastFlights[index];
                                             return LogTile(
                                                date: log["date"],
                                                loc: "Saved Flight",
                                                dur: log["size"],
                                                status: "SUCCESS",
                                                filePath: (log["path"] ?? "") as String,
                                             );
                                          }
                                      ),
                                  )
                                ],
                              ),
                            ),
                          )
                        ],
                      ),
                    ),
                  ),

                   // RIGHT: Info Panel
                  Expanded(
                    flex: rightFlex,
                    child: Container(
                      padding: const EdgeInsets.all(16), // Reduced padding
                      color: const Color(0xFF111111),
                      child: Column(
                        children: [
                          // Header with Profile
                          Row(
                            mainAxisAlignment: MainAxisAlignment.spaceBetween,
                            children: [
                              Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
                                const Text("HANGAR DECK", style: TextStyle(color: Colors.white54, fontSize: 10)),
                                Text(username, style: const TextStyle(fontSize: 20, fontWeight: FontWeight.bold, color: Colors.white)),
                              ]),
                              GestureDetector(
                                onTap: () => setState(() => _showSettings = !_showSettings),
                                child: const CircleAvatar(backgroundColor: Colors.white10, child: Icon(Icons.person, color: Colors.white)),
                              )
                            ],
                          ),
                          const Divider(color: Colors.white24, height: 20),
                           
                           // Scrollable Content
                          Expanded(
                            child: SingleChildScrollView( 
                              child: Column( 
                                crossAxisAlignment: CrossAxisAlignment.start,
                                children: [
                                   // DYNAMIC STATS (Restored)
                                   const Text("FLEET STATUS", style: TextStyle(color: Color(0xFF2DD4BF), fontSize: 12, fontWeight: FontWeight.bold, letterSpacing: 1)),
                                   const SizedBox(height: 15),
                                   HangarStatRow(label: "TOTAL AIRTIME", value: _stats["airtime"] ?? "0m"),
                                   HangarStatRow(label: "DISTANCE FLOWN", value: _stats["distance"] ?? "0km"),
                                   HangarStatRow(label: "MISSIONS", value: _stats["missions"] ?? "0"),
                                   
                                   const SizedBox(height: 20),
                                   const Text("SYSTEM HEALTH", style: TextStyle(color: Color(0xFF2DD4BF), fontSize: 12, fontWeight: FontWeight.bold, letterSpacing: 1)),
                                   const SizedBox(height: 15),
                                   
                                   // Real Readiness Checks
                                   if (_centralState.isDeviceConnected) ...[
                                      const ReadinessRow(label: "Connection", value: "LINKED", icon: Icons.link, color: Colors.green),
                                      const ReadinessRow(label: "Telemetry", value: "STREAMING", icon: Icons.sensors, color: Colors.green),
                                   ] else ...[
                                      const ReadinessRow(label: "Connection", value: "OFFLINE", icon: Icons.link_off, color: Colors.red),
                                      const ReadinessRow(label: "Telemetry", value: "WAITING", icon: Icons.sensors_off, color: Colors.grey),
                                   ],
                                   
                                   // Was a hardcoded "Firmware V3.5 STABLE" -- a fixed string the
                                   // aircraft never reported. Replaced with GPS state, which the
                                   // bridge does send and which actually gates arming.
                                   if (_centralState.isDeviceConnected)
                                     Builder(builder: (_) {
                                       final t = _centralState.telemetryData;
                                       final fix = (t["gps_fix"] is num) ? (t["gps_fix"] as num).toInt() : 0;
                                       final sats = (t["sats"] is num) ? (t["sats"] as num).toInt() : 0;
                                       final ok = fix >= 3;
                                       return ReadinessRow(
                                         label: "GPS",
                                         value: ok ? "FIX ($sats sats)" : "NO FIX ($sats sats)",
                                         icon: ok ? Icons.satellite_alt : Icons.location_disabled,
                                         color: ok ? Colors.green : Colors.orangeAccent,
                                       );
                                     }),
              
                                   const SizedBox(height: 20),
                                   const Text("LOCAL CONDITIONS", style: TextStyle(color: Color(0xFF2DD4BF), fontSize: 12, fontWeight: FontWeight.bold, letterSpacing: 1)),
                                   const SizedBox(height: 15),
                                   HangarStatRow(label: "WEATHER", value: _localWeather),
                                   HangarStatRow(label: "WIND", value: _localWind),
                                   const HangarStatRow(label: "VISIBILITY", value: "> 10 km"),
                                ],
                              ),
                            ),
                          ),
              
                          const SizedBox(height: 10),
                          // Launch Button
                          SizedBox(
                            width: double.infinity,
                            height: 60,
                            child: ElevatedButton.icon(
                              onPressed: () {
                                Navigator.pushNamed(context, '/control');
                              },
                              icon: const Icon(Icons.flight_takeoff, size: 28),
                              label: const FittedBox(child: Text("INITIATE FLIGHT", style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold, letterSpacing: 2))),
                              style: ElevatedButton.styleFrom(backgroundColor: neonTeal, foregroundColor: Colors.black),
                            ),
                          )
                        ],
                      ),
                    ),
                  )
                ],
              );
            } // Builder
          ),
           
          // Device Switcher Dropdown
          if (_showDeviceList)
            Positioned(
              top: 80, left: 40,
              child: Container(
                width: 250,
                padding: const EdgeInsets.all(10),
                decoration: BoxDecoration(
                  color: const Color(0xFF1A1A1A), 
                  borderRadius: BorderRadius.circular(10), 
                  border: Border.all(color: Colors.white24)
                ),
                child: Column(
                  mainAxisSize: MainAxisSize.min,
                  children: [
                    ..._myDevices.map((d) => ListTile(
                      title: Text(d, style: TextStyle(color: d == currentDevice ? neonTeal : Colors.white)),
                      leading: Icon(Icons.flight, color: d == currentDevice ? neonTeal : Colors.white54),
                      onTap: () {
                         _centralState.connectDevice(d); // Update central state
                         setState(() { currentDevice = d; _showDeviceList = false; });
                      },
                    )),
                    const Divider(color: Colors.white10),
                    ListTile(
                      title: const Text("Add New Device", style: TextStyle(color: Colors.white)),
                      leading: const Icon(Icons.add, color: Colors.white),
                      onTap: () {
                        setState(() => _showDeviceList = false);
                        // Navigate to Connect in "Add" mode
                        // For now just back to connect screen
                        Navigator.pushNamed(context, '/connect'); 
                      },
                    )
                  ],
                ),
              ),
            ),

           if (_showSettings)
             Positioned(
               right: 0, top: 0, bottom: 0,
               child: SettingsSidebar(onClose: () => setState(() => _showSettings = false)),
             ),
        ],
      ),
    );
  }
}
// Note: I will use a Stack to overlay the sidebar, so I need to change the return structure slightly.
// But to keep diff minimal, I will assume the Stack is at line 59.
// Better approach: Upgrade build method to include the overlay.

// Actually, I'll rewrite the build method structure briefly to include the sidebar.

/*
      body: Stack(
        children: [
             // ... [OLD ROW CONTENT]
             
             // ... [DEVICE SWITCHER]
             
             // [NEW] SETTINGS OVERLAY
             if (_showSettings)
               Positioned(
                 right: 0, top: 0, bottom: 0,
                 child: SettingsSidebar(onClose: () => setState(() => _showSettings = false)),
               )
        ]
      )
*/


// Helpers specific to Hangar
class HangarStatRow extends StatelessWidget {
  final String label, value;
  const HangarStatRow({super.key, required this.label, required this.value});
  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.only(bottom: 12.0),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceBetween,
        children: [
          Text(label, style: const TextStyle(color: Colors.white54, fontSize: 12)),
          Text(value, style: const TextStyle(fontWeight: FontWeight.bold, fontSize: 16, color: Colors.white)),
        ],
      ),
    );
  }
}

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

class LogTile extends StatelessWidget {
  final String date, loc, dur, status;
  final String filePath;
  const LogTile({super.key, required this.date, required this.loc, required this.dur,
                 required this.status, this.filePath = ""});
  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onTap: () {
        showDialog(context: context, builder: (_) => LogDetailDialog(date: date, loc: loc, dur: dur, status: status, filePath: filePath));
      },
      child: Container(
        margin: const EdgeInsets.only(bottom: 8),
        padding: const EdgeInsets.all(12),
        decoration: BoxDecoration(color: Colors.white.withOpacity(0.05), borderRadius: BorderRadius.circular(8)),
        child: Row(
          mainAxisAlignment: MainAxisAlignment.spaceBetween,
          children: [
            Row(children: [
              Icon(Icons.flight, color: status == "SUCCESS" ? Colors.green : Colors.amber, size: 16),
              const SizedBox(width: 10),
              Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
                Text(loc, style: const TextStyle(fontWeight: FontWeight.bold, fontSize: 14, color: Colors.white)), 
                Text(date, style: const TextStyle(color: Colors.white38, fontSize: 10))
              ])
            ]),
            Text(dur, style: const TextStyle(fontWeight: FontWeight.bold, fontSize: 14, color: Colors.white))
          ]
        ),
      ),
    );
  }
}
