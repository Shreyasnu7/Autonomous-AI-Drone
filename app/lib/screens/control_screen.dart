import 'dart:math';
import 'package:flutter/material.dart';
import 'package:google_maps_flutter/google_maps_flutter.dart';
import 'package:provider/provider.dart';
import 'dart:io'; 
import 'dart:ui' as ui;
import 'dart:convert';
import 'package:image_picker/image_picker.dart';
import '../services/central_state.dart';
import '../services/api_service.dart';
import '../widgets/shared_ui.dart';
import '../widgets/settings_sidebar.dart';
import '../widgets/analytics_sidebar.dart';
import '../widgets/cam_button.dart';
import '../theme/app_theme.dart';
import '../widgets/video_feed_widget.dart'; // Robust Video Player

class ControlScreen extends StatefulWidget {
  const ControlScreen({super.key});

  @override
  State<ControlScreen> createState() => _ControlScreenState();
}

class _ControlScreenState extends State<ControlScreen> with TickerProviderStateMixin {
  // Telemetry Notifiers (Optimization: No setState on every packet)
  final ValueNotifier<double> altitudeVN = ValueNotifier(0.0);
  final ValueNotifier<double> speedVN = ValueNotifier(0.0);
  final ValueNotifier<double> batteryVN = ValueNotifier(0.0);
  final ValueNotifier<int> satsVN = ValueNotifier(12);
  final ValueNotifier<double> distanceVN = ValueNotifier(0.0);
  final ValueNotifier<List<String>> logsVN = ValueNotifier(["> SYSTEM READY"]);
  final ValueNotifier<String?> aiResponseVN = ValueNotifier(null); // Real AI Response
  // Attitude
  final ValueNotifier<double> rollVN = ValueNotifier(0.0);
  final ValueNotifier<double> pitchVN = ValueNotifier(0.0);
  final ValueNotifier<double> yawVN = ValueNotifier(0.0);
  final ValueNotifier<double> headingVN = ValueNotifier(0.0); // Heading in Degrees
  
  GoogleMapController? _mapController;
  final ValueNotifier<LatLng> _droneLocationVN = ValueNotifier(const LatLng(37.42796133580664, -122.085749655962)); 
  BitmapDescriptor? _droneMarker;

  // UI State
  String flightMode = "P-GPS";
  bool _isMapExpanded = false; // Minimap default
  bool _isMissionMode = false;
  final List<LatLng> _missionWaypoints = []; // P1.1: Use Real LatLng
  bool _showSettingsSidebar = false;  
  bool _showAnalyticsSidebar = false;
  bool _isRecording = false;
  
  // Joysticks
  Offset leftStick = Offset.zero;
  Offset rightStick = Offset.zero;
  final TextEditingController _aiController = TextEditingController();
  final ValueNotifier<int> _videoRefreshVN = ValueNotifier(0); // Video Retry Logic
  
  late AnimationController _modelRotator;

  @override
  void initState() {
    super.initState();
    _modelRotator = AnimationController(vsync: this, duration: const Duration(seconds: 8))..repeat();
    
    // Generate Drone Marker
    _createDroneMarker();

    // Auto-Follow Listener
    _droneLocationVN.addListener(() {
      if (_mapController != null && !_isMapExpanded) {
         _mapController!.animateCamera(CameraUpdate.newLatLng(_droneLocationVN.value));
      }
    });

    // Listen to real telemetry
    final state = context.read<CentralState>();

    // AUTO-CONNECT on entering control screen (direct modes: tailscale/local)
    final cmode = state.config['connection_mode'] ?? 'cloud';
    if (cmode == 'tailscale' || cmode == 'local') {
       state.connectDevice(state.config['drone_id'] ?? 'drone1');
       final dip = cmode == 'tailscale' ? state.config['tailscale_ip'] : state.config['local_ip'];
       WidgetsBinding.instance.addPostFrameCallback((_) {
          if (mounted) ScaffoldMessenger.of(context).showSnackBar(
            SnackBar(content: Text("Linking drone: $cmode -> ws://$dip:8000"),
                     backgroundColor: Colors.blueGrey, duration: const Duration(seconds: 4)));
       });
    } else {
       WidgetsBinding.instance.addPostFrameCallback((_) {
          if (mounted) ScaffoldMessenger.of(context).showSnackBar(
            const SnackBar(content: Text("Mode=CLOUD: not linking directly. Pick TAILSCALE."),
                     backgroundColor: Colors.orange, duration: Duration(seconds: 5)));
       });
    }

    // Note: We use the stream we created in TelemetryService
    state.telemetry.telemetryStream.listen((data) {
       if (!mounted) return;
       
       // Update Notifiers (Optimized UI updates)
       if (data.containsKey("altitude")) altitudeVN.value = (data["altitude"] ?? 0.0).toDouble();
       if (data.containsKey("speed")) speedVN.value = (data["speed"] ?? 0.0).toDouble();
       if (data.containsKey("battery")) batteryVN.value = (data["battery"] ?? 0.0).toDouble();
       if (data.containsKey("distance")) distanceVN.value = (data["distance"] ?? 0.0).toDouble(); // REAL DISTANCE
       if (data.containsKey("sats")) satsVN.value = (data["sats"] ?? 0).toInt();
       
       // ATTITUDE (Roll/Pitch/Yaw for 3D Model)
       // Assuming data has 'attitude': {'roll': ..., 'pitch': ..., 'yaw': ...} or flat fields
       if (data.containsKey("roll")) rollVN.value = (data["roll"] ?? 0.0).toDouble();
       if (data.containsKey("pitch")) pitchVN.value = (data["pitch"] ?? 0.0).toDouble();
       if (data.containsKey("yaw")) yawVN.value = (data["yaw"] ?? 0.0).toDouble();
       
       if (data.containsKey("heading")) headingVN.value = (data["heading"] ?? 0.0).toDouble();

       if (data.containsKey("lat")) {
           _droneLocationVN.value = LatLng(data["lat"], data["lng"]);
       }
    });

    // LISTEN FOR AI DIRECTOR RESPONSE (the laptop's reasoning, back over Tailscale)
    state.telemetry.aiResponseStream.listen((resp) {
       if (!mounted) return;
       final thought = (resp['thought'] ?? resp['reasoning'] ?? '').toString();
       final action = (resp['action'] ?? '').toString();
       final style = (resp['style'] ?? '').toString();
       final parts = <String>[];
       if (thought.isNotEmpty) parts.add("🧠 $thought");
       if (action.isNotEmpty) parts.add("🚀 ${action.toUpperCase()}");
       if (style.isNotEmpty) parts.add("🎨 $style");
       aiResponseVN.value = parts.isEmpty ? "DONE" : parts.join("\n");
    });

    // LISTEN FOR CRITICAL ALERTS
    String? _lastAlertMsg;
    DateTime _lastAlertTime = DateTime.now();

    state.telemetry.alertStream.listen((alert) {
       if (!mounted) return;
       final msg = alert['payload']?['msg'] ?? alert['msg'] ?? "ALERT"; // Handle nested or flat
       final level = alert['payload']?['level'] ?? alert['level'] ?? "info";
       
       // MAP LOGIC: Auto-Clear Mission Points
       if (msg.toString().contains("Mission Complete") || msg.toString().contains("MISSION_COMPLETE")) {
           setState(() {
               _missionWaypoints.clear();
               _isMissionMode = false;
               _addLog("✅ MISSION DATA CLEARED");
           });
       }

       _addLog("⚠ $msg");
       
       // DEDUPLICATION: Don't show same SnackBar within 2 seconds
       if (_lastAlertMsg == msg && DateTime.now().difference(_lastAlertTime).inSeconds < 2) return;
       _lastAlertMsg = msg;
       _lastAlertTime = DateTime.now();
       
       // UI NOISE REDUCTION: Only show SnackBar for CRITICAL/EMERGENCY
       if (level != 'critical' && level != 'emergency') return;

       Color snackColor = Colors.redAccent;
       
       ScaffoldMessenger.of(context).clearSnackBars(); // Prevent stacking
       ScaffoldMessenger.of(context).showSnackBar(
         SnackBar(
             padding: EdgeInsets.symmetric(horizontal: 16, vertical: 0), // Minimal padding
             content: SizedBox(
               height: 20, // Strict Height constraint
               child: SingleChildScrollView(
                 scrollDirection: Axis.horizontal,
                 child: Row(
                   children: [
                     Icon(Icons.warning_amber, color: Colors.white, size: 16), 
                     SizedBox(width: 8), 
                     Text(msg.toString().toUpperCase(), style: TextStyle(fontWeight: FontWeight.bold, fontSize: 12), softWrap: false)
                   ]
                 ),
               ),
             ),
             backgroundColor: snackColor,
             behavior: SnackBarBehavior.floating,
             shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(20)),
             margin: EdgeInsets.only(top: 20, left: 20, right: 20, bottom: MediaQuery.of(context).size.height - 80), 
             duration: Duration(seconds: 3),
         )
       );
    });
  }

  Future<void> _createDroneMarker() async {
    final recorder = ui.PictureRecorder();
    final canvas = Canvas(recorder);
    const size = Size(80, 80); 
    
    // Config
    final bodyPaint = Paint()..color = Colors.black..style = PaintingStyle.fill;
    final outlinePaint = Paint()..color = Colors.white..style = PaintingStyle.stroke..strokeWidth = 2; // White Outline
    final armPaint = Paint()..color = Colors.black..strokeWidth = 4..style = PaintingStyle.stroke..strokeCap = StrokeCap.round; // Thicker arms
    final armOutline = Paint()..color = Colors.white..strokeWidth = 6..style = PaintingStyle.stroke..strokeCap = StrokeCap.round; // Arm Outline
    final arrowPaint = Paint()..color = Colors.redAccent..style = PaintingStyle.fill;
    
    // 1. Draw Arm Outlines (Background)
    canvas.drawLine(const Offset(20, 20), const Offset(60, 60), armOutline);
    canvas.drawLine(const Offset(60, 20), const Offset(20, 60), armOutline);

    // 2. Draw Arms (Foreground)
    canvas.drawLine(const Offset(20, 20), const Offset(60, 60), armPaint);
    canvas.drawLine(const Offset(60, 20), const Offset(20, 60), armPaint);
    
    // 3. Draw Body & Props (With Outlines)
    // Props
    for (final offset in [const Offset(20, 20), const Offset(60, 20), const Offset(60, 60), const Offset(20, 60)]) {
       canvas.drawCircle(offset, 9, outlinePaint); // Outline
       canvas.drawCircle(offset, 8, bodyPaint);    // Fill
    }

    // Body
    canvas.drawCircle(const Offset(40, 40), 13, outlinePaint);
    canvas.drawCircle(const Offset(40, 40), 12, bodyPaint); 

    // 4. Direction Arrow
    final p = Path()..moveTo(40, 15)..lineTo(50, 35)..lineTo(30, 35)..close();
    canvas.drawPath(p, arrowPaint);
    canvas.drawPath(p, Paint()..color = Colors.white..style = PaintingStyle.stroke..strokeWidth = 1); // Arrow Outline

    final picture = recorder.endRecording();
    final img = await picture.toImage(80, 80);
    final byteData = await img.toByteData(format: ui.ImageByteFormat.png);
    
    if (byteData != null) {
      if (mounted) setState(() => _droneMarker = BitmapDescriptor.fromBytes(byteData.buffer.asUint8List()));
    }
  }

  @override
  void dispose() {
    altitudeVN.dispose();
    speedVN.dispose();
    batteryVN.dispose();
    satsVN.dispose();
    distanceVN.dispose();
    logsVN.dispose();
    
    _aiController.dispose();
    _modelRotator.dispose();
    super.dispose();
  }

  void _addLog(String msg) {
    final list = List<String>.from(logsVN.value);
    list.insert(0, "> $msg");
    if (list.length > 6) list.removeLast();
    logsVN.value = list;
  }

  // AI State
  String _selectedProvider = "gemini";
  double _aiSensitivity = 0.5;
  List<String> _mediaFiles = []; // List of Base64 strings
  List<String> _mediaTypes = []; // 'image', 'video'
  
  Future<void> _pickMedia(Function(void Function()) modalSetState) async {
     try {
       final ImagePicker picker = ImagePicker();
       final List<XFile> images = await picker.pickMultiImage();
       
       if (images.isNotEmpty) {
         List<String> newFiles = [];
         List<String> newTypes = [];
         
         // Batch Process
         for (var img in images) {
             final bytes = await img.readAsBytes();
             newFiles.add(base64Encode(bytes));
             newTypes.add('image');
         }

         // Single Update
         setState(() {
           _mediaFiles.addAll(newFiles);
           _mediaTypes.addAll(newTypes);
         });
         modalSetState(() {}); 
         
         _addLog("ATTACHED ${images.length} IMAGES");
       }
     } catch (e) {
       _addLog("PICKER ERROR: $e");
     }
  }

  Future<void> _pickVideo(Function(void Function()) modalSetState) async {
      try {
        final ImagePicker picker = ImagePicker();
        final XFile? video = await picker.pickVideo(source: ImageSource.gallery);
        
        if (video != null) {
            final bytes = await video.readAsBytes();
            setState(() {
              _mediaFiles.add(base64Encode(bytes));
              _mediaTypes.add('video');
            });
            modalSetState(() {});
            _addLog("ATTACHED VIDEO");
        }
      } catch (e) {
         _addLog("VIDEO PICK ERROR: $e");
      }
  }

  void _executeAICommand(String command) {
    if (command.trim().isEmpty) return;
    _addLog("AI: SENDING COMMAND...");
    
    // Construct Rich Payload
    final config = context.read<CentralState>().config;
    final keys = {
      "gemini": config["gemini_api_key"] ?? "", 
      "openai": config["openai_api_key"] ?? ""
    };

    final payload = {
       "text": command,
       "provider": _selectedProvider,
       "camera": "external", 
       "sensitivity": _aiSensitivity,
       // LINKED AI: We now send the LIVE telemetry so the Brain isn't blind!
       "telemetry": context.read<CentralState>().telemetryData, 
       // EXISTING:
       "media": List.generate(_mediaFiles.length, (i) => {
          "type": _mediaTypes[i],
          "data": _mediaFiles[i] 
       }),
       "api_keys": keys, 
       "timestamp": DateTime.now().millisecondsSinceEpoch
    };

    // DIRECT (Tailscale/Local): send the AI job straight to the LAPTOP AI over the live WS.
    // The bridge relays type 'ai_job' to laptop_vision, which runs the Gemini+Qwen director.
    // Cloud mode (or if the WS isn't live) falls back to the HTTP endpoint.
    final mode = (config['connection_mode'] ?? 'cloud').toString();
    bool sentDirect = false;
    if (mode == 'tailscale' || mode == 'local') {
      sentDirect = context.read<CentralState>().telemetry.sendAiJob(payload);
    }
    if (sentDirect) {
      _addLog("AI: SENT TO LAPTOP (Tailscale)");
    } else {
      _sendAIRequestViaHttp(payload);
    }

    setState(() {
      flightMode = "AI-PILOT";
      _aiController.clear();
      _mediaFiles.clear(); // Clear after send
      _mediaTypes.clear();
      aiResponseVN.value = "PROCESSING...";
    });
  }

  Future<void> _sendAIRequestViaHttp(Map<String, dynamic> payload) async {
      try {
         aiResponseVN.value = "CONNECTING TO ${_selectedProvider.toUpperCase()}...";
         
         // Use the real API Service
         final resp = await ApiService.post("/director/ai/command", payload);
         
         // Parse response
         if (resp != null && resp.containsKey("status") && resp["status"].toString().contains("queued")) {
             _addLog("CMD QUEUED [${_selectedProvider}]");
             
             if (resp.containsKey("plan")) {
                final plan = resp["plan"];
                final reasoning = plan["reasoning"] ?? plan["thought_process"] ?? "Executing command...";
                final action = plan["action"] ?? "UNKNOWN";
                
                // Format nicely
                final msg = "🧠 THOUGHT: $reasoning\n🚀 ACTION: ${action.toString().toUpperCase()}";
                aiResponseVN.value = msg;
                
             } else {
                aiResponseVN.value = "INSTRUCTION RECEIVED";
             }

         } else {
             // If status is "executed" or something else success-like
             if (resp != null && resp.containsKey("plan")) {
                  final plan = resp["plan"];
                  final action = plan["action"] ?? "UNKNOWN";
                  aiResponseVN.value = "🚀 EXECUTING: $action";
             } else {
                  aiResponseVN.value = "SERVER: ${resp.toString()}";
             }
         }

      } catch (e) {
         _addLog("AI ERROR: $e");
         aiResponseVN.value = "NET ERROR: $e";
      }
  }

  // Called 10-60 times a second by joystick widgets
  void _onJoystickUpdate(double x, double y, double z, double r) {
     context.read<CentralState>().telemetry.sendJoystick(x, y, z, r);
  }

  void _toggleMap() {
    setState(() {
      _isMapExpanded = !_isMapExpanded;
      if (!_isMapExpanded) _isMissionMode = false; 
    });
  }


  
  // DISARM confirmation popup -> on OK, force-disarm (cuts motors). Cancel does nothing.
  void _confirmDisarm(BuildContext context) {
    showDialog(
      context: context,
      barrierColor: Colors.black87,
      builder: (c) => AlertDialog(
        backgroundColor: const Color(0xFF111111),
        shape: RoundedRectangleBorder(
          borderRadius: BorderRadius.circular(16),
          side: const BorderSide(color: Colors.redAccent),
        ),
        title: Row(children: const [
          Icon(Icons.warning_amber_rounded, color: Colors.redAccent),
          SizedBox(width: 10),
          Text("Disarm?", style: TextStyle(color: Colors.white)),
        ]),
        content: const Text(
          "Are you sure you want to disarm?\nThis cuts the motors immediately.",
          style: TextStyle(color: Colors.white70),
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(c).pop(),
            child: const Text("CANCEL", style: TextStyle(color: Colors.white54)),
          ),
          TextButton(
            style: TextButton.styleFrom(backgroundColor: Colors.redAccent),
            onPressed: () {
              Navigator.of(c).pop();
              _addLog("DISARM!");
              context.read<CentralState>().telemetry.sendCommand("DISARM");
            },
            child: const Text("OK", style: TextStyle(color: Colors.white, fontWeight: FontWeight.bold)),
          ),
        ],
      ),
    );
  }

  void _showAIModal(BuildContext context) {
    final color = const Color(0xFF2DD4BF);
    _aiController.clear();
    showDialog(
      context: context, 
      barrierColor: Colors.black87, 
      builder: (c) => StatefulBuilder( // FIX THUMBNAILS: Rebuilds dialog when state changes
        builder: (context, modalSetState) {
          return AlertDialog(
            backgroundColor: const Color(0xFF111111), 
            scrollable: true, 
            shape: RoundedRectangleBorder(
              borderRadius: BorderRadius.circular(20), 
              side: BorderSide(color: color.withOpacity(0.5))
            ), 
            title: Row(children: [Icon(Icons.psychology, color: color), const SizedBox(width: 10), const Text("Neural Command", style: TextStyle(color: Colors.white))]), 
            content: Column(
              mainAxisSize: MainAxisSize.min, 
              children: [
                // LIVE AI FEEDBACK IN MODAL
                // TERMINAL OUTPUT BOX
                Container(
                  height: 150, // REVERTED: Original Height
                  width: double.maxFinite,
                  margin: const EdgeInsets.only(bottom: 15),
                  padding: const EdgeInsets.all(12),
                  decoration: BoxDecoration(
                    color: Colors.black,
                    borderRadius: BorderRadius.circular(10),
                    border: Border.all(color: color.withOpacity(0.3)),
                    boxShadow: [BoxShadow(color: color.withOpacity(0.05), blurRadius: 10)]
                  ),
                  child: ValueListenableBuilder<String?>(
                    valueListenable: aiResponseVN,
                    builder: (context, msg, _) {
                       if (msg == null) {
                         return Center(
                           child: Column(
                             mainAxisAlignment: MainAxisAlignment.center,
                             children: [
                               Icon(Icons.hub, color: Colors.white12, size: 40),
                               const SizedBox(height: 10),
                               const Text("NEURAL LINK ACTIVE\nAwaiting Query...", textAlign: TextAlign.center, style: TextStyle(color: Colors.white24, fontSize: 10, fontFamily: 'Courier'))
                             ],
                           )
                         );
                       }
                       return SingleChildScrollView(
                         child: Text(
                           "> $msg", 
                           style: TextStyle(color: color, fontFamily: 'Courier', fontSize: 12, height: 1.5)
                         )
                       );
                    }
                  ),
                ),
                
                // INPUT ROW
                Row(
                  children: [
                    Expanded(
                      child: TextField(
                        controller: _aiController, 
                        autofocus: true, 
                        style: const TextStyle(color: Colors.white, fontFamily: 'Courier'), 
                        decoration: InputDecoration(
                          hintText: "Enter command...", 
                          hintStyle: const TextStyle(color: Colors.white30, fontFamily: 'Courier'), 
                          filled: true, 
                          fillColor: Colors.white10, 
                          contentPadding: const EdgeInsets.symmetric(horizontal: 15, vertical: 0),
                          border: OutlineInputBorder(borderRadius: BorderRadius.circular(8), borderSide: BorderSide.none), 
                        ), 
                        onSubmitted: (val) { 
                          _executeAICommand(val); 
                          _aiController.clear();
                        }
                      ),
                    ),
                    const SizedBox(width: 5),
                    
                    // PREVIEW CAROUSEL
                    if (_mediaFiles.isNotEmpty) 
                      Container(
                        height: 50,
                        margin: const EdgeInsets.only(right: 5),
                        width: 100, // Slightly tighter
                        child: ListView.separated(
                          scrollDirection: Axis.horizontal,
                          itemCount: _mediaFiles.length,
                          separatorBuilder: (_,__) => const SizedBox(width: 5),
                          itemBuilder: (context, index) {
                             return Stack(
                               children: [
                                 Container(
                                   width: 50, height: 50,
                                   decoration: BoxDecoration(
                                     border: Border.all(color: AppTheme.neonTeal),
                                     borderRadius: BorderRadius.circular(5),
                                     image: _mediaTypes[index] == 'image'
                                        ? DecorationImage(image: MemoryImage(base64Decode(_mediaFiles[index])), fit: BoxFit.cover)
                                        : null
                                   ),
                                   child: _mediaTypes[index] == 'video' ? const Center(child: Icon(Icons.videocam, color: Colors.white, size: 20)) : null,
                                 ),
                                 Positioned(
                                   right: 0, top: 0,
                                   child: GestureDetector(
                                     onTap: () {
                                         // Update BOTH states to ensure sync
                                         setState(() { _mediaFiles.removeAt(index); _mediaTypes.removeAt(index); });
                                         modalSetState(() {});
                                     },
                                     child: Container(color: Colors.black54, child: const Icon(Icons.close, size: 14, color: Colors.red)),
                                   ),
                                 )
                               ],
                             );
                          }
                        ),
                      ),

                    // BUTTONS COLUMN (Photo + Video)
                    Column(
                      mainAxisSize: MainAxisSize.min,
                      children: [
                        InkWell(
                          onTap: () => _pickMedia(modalSetState),
                          child: Icon(Icons.image, color: _mediaFiles.isNotEmpty ? AppTheme.neonTeal : Colors.white54, size: 20),
                        ),
                        const SizedBox(height: 8),
                         InkWell(
                          onTap: () => _pickVideo(modalSetState),
                          child: Icon(Icons.videocam, color: Colors.white54, size: 20),
                        ),
                      ],
                    ),
                    const SizedBox(width: 5),
                    
                    IconButton.filled(
                      style: IconButton.styleFrom(backgroundColor: color, foregroundColor: Colors.black, shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(8))),
                      icon: const Icon(Icons.send), 
                      onPressed: () { 
                        _executeAICommand(_aiController.text); 
                        _aiController.clear();
                      }
                    )
                  ],
                ), 
                const SizedBox(height: 15), 
                // AI PROVIDER SELECTION
                Row(mainAxisAlignment: MainAxisAlignment.spaceAround, children: [
                   ChoiceChip(
                     label: const Text("OpenAI"), 
                     selected: _selectedProvider == "openai", 
                     onSelected: (v) => modalSetState(() => _selectedProvider = "openai"),
                     selectedColor: color,
                     backgroundColor: Colors.white10,
                   ),
                   ChoiceChip(
                     label: const Text("Gemini"), 
                     selected: _selectedProvider == "gemini", 
                     onSelected: (v) => modalSetState(() => _selectedProvider = "gemini"),
                     selectedColor: color,
                     backgroundColor: Colors.white10,
                   )
                ]),
                const SizedBox(height: 10),
                SizedBox(
                  height: 40, 
                  width: 300, 
                  child: ListView(
                    scrollDirection: Axis.horizontal, 
                    children: [
                       _AIChip(label: "Follow Me", color: color, onTap: () { _executeAICommand("FOLLOW ME"); }), 
                       _AIChip(label: "Orbit", color: color, onTap: () { _executeAICommand("ORBIT"); }), 
                       _AIChip(label: "Cinematic", color: color, onTap: () { _executeAICommand("CINEMATIC MODE"); })
                    ]
                  )
                )
              ]
            )
          );
        }
      )
    ).then((_) {
       // START FADE TIMER (3s after close)
       if (aiResponseVN.value != null) {
          final capture = aiResponseVN.value;
          Future.delayed(const Duration(seconds: 3), () {
             if (mounted && aiResponseVN.value == capture) {
                aiResponseVN.value = null;
             }
          });
       }
    });
  }

  @override
  Widget build(BuildContext context) {
    final size = MediaQuery.of(context).size;
    final neonTeal = const Color(0xFF2DD4BF);
    final neonRed = const Color(0xFFF43F5E);
    final neonYellow = const Color(0xFFFACC15);
    final neonBlue = const Color(0xFF3B82F6);

    return Scaffold(
      resizeToAvoidBottomInset: false, 
      backgroundColor: Colors.black,
      body: LayoutBuilder(
        builder: (context, constraints) {
          final isLandscape = constraints.maxWidth > constraints.maxHeight;
          
          // Smart Scaling: Use percentage but CAP at reasonable max sizes for Tablets
          final rawJoySize = isLandscape ? constraints.maxHeight * 0.28 : constraints.maxWidth * 0.28;
          final joystickSize = rawJoySize > 220 ? 220.0 : rawJoySize; // Max 220px joystick
          
          final rawBtnSize = isLandscape ? constraints.maxHeight * 0.09 : constraints.maxWidth * 0.09;
          final btnSize = rawBtnSize > 70 ? 70.0 : rawBtnSize; // Max 70px buttons (standard FAB is 56)
          
          return Stack(
            fit: StackFit.expand,
            children: [
                // Background
                Image.network(
                  'https://images.unsplash.com/photo-1473968512647-3e447244af8f?q=80&w=2070&auto=format&fit=crop', 
                  fit: BoxFit.cover, 
                  errorBuilder: (c,e,s) => Container(color: const Color(0xFF050505))
                ),
            
                // Video Feed Layer



            // VIDEO FEED LAYER (Underneath UI, but above generic background)
            Positioned.fill(
               child: Container(
                 color: const Color(0xFF050505),
                 child: Stack(
                   fit: StackFit.expand,
                   children: [
                    // SETTINGS OVERLAY
             if (_showSettingsSidebar)
               Positioned(
                 right: 0, top: 0, bottom: 0,
                 child: SettingsSidebar(
                    onClose: () => setState(() => _showSettingsSidebar = false),
                 ),
               ),
                      // V85: ROBUST VIDEO PLAYER (Using Polling Widget)
                      const VideoFeed(),

                       // Map (Collapsible) - MOVED BEHIND CONTROLS
            AnimatedPositioned(
             duration: const Duration(milliseconds: 400),
             curve: Curves.easeInOut,
             top: _isMapExpanded ? 0 : 60, bottom: _isMapExpanded ? 0 : null, 
             right: _isMapExpanded ? 0 : 20, left: _isMapExpanded ? 0 : null,   
             width: _isMapExpanded ? null : isLandscape ? size.width * 0.2 : 140, 
             height: _isMapExpanded ? null : isLandscape ? size.height * 0.3 : 140, 
             child: GestureDetector(
               onTap: _isMapExpanded ? null : _toggleMap, 
               child: Container(
                 decoration: BoxDecoration(color: const Color(0xFF111111), border: Border.all(color: _isMapExpanded ? neonTeal : Colors.white24, width: _isMapExpanded ? 2 : 1), borderRadius: BorderRadius.circular(_isMapExpanded ? 0 : 10)),
                 child: ClipRRect(
                   borderRadius: BorderRadius.circular(_isMapExpanded ? 0 : 10),
                   child: Stack(
                     children: [
                       // REAL GOOGLE MAP (Platform Safe)
                       Builder(builder: (c) {
                         try {
                            // Google Maps only works on Mobile/Web (mostly)
                            // We explicitly avoid it on Desktop to prevent crashes or empty views
                            bool showMap = false;
                            try {
                               // Simple platform check (Platform from dart:io)
                               showMap = !Platform.isLinux && !Platform.isWindows && !Platform.isMacOS;
                            } catch (e) {
                               showMap = true; // Likely web
                            }
                            
                            if (!showMap) {
                               return Container(
                                 color: Colors.black87, 
                                 child: Center(
                                   child: Column(
                                     mainAxisSize: MainAxisSize.min,
                                     children: [
                                       const Icon(Icons.map_outlined, color: Colors.white24, size: 40),
                                       const SizedBox(height: 10),
                                       const Text("MAPS NOT SUPPORTED\nON DESKTOP", textAlign: TextAlign.center, style: TextStyle(color: Colors.white30))
                                     ]
                                   )
                                 )
                               );
                            }

                            return ValueListenableBuilder<LatLng>(
                             valueListenable: _droneLocationVN,
                             builder: (context, pos, _) {
                               return GoogleMap(
                                 mapType: MapType.hybrid,
                                 initialCameraPosition: CameraPosition(target: pos, zoom: 18),
                                 onMapCreated: (c) => _mapController = c,
                                 onTap: (latlng) {
                                    if (_isMissionMode) {
                                       setState(() => _missionWaypoints.add(latlng)); // P1.1: Store LatLng directly
                                    }
                                 },
                                 polylines: {
                                   if (_missionWaypoints.isNotEmpty)
                                     Polyline(
                                       polylineId: const PolylineId("mission_path"),
                                       points: _missionWaypoints, // P1.1: Already LatLng
                                       color: neonYellow,
                                       width: 3,
                                     )
                                 },
                                 markers: {
                                    Marker(
                                      markerId: const MarkerId("drone"), 
                                      position: pos, 
                                      // Rotate flat if we had heading, using groundAnchor usually
                                      anchor: const Offset(0.5, 0.5),
                                      rotation: headingVN.value, 
                                      icon: _droneMarker ?? BitmapDescriptor.defaultMarkerWithHue(BitmapDescriptor.hueCyan)
                                    ),
                                    ..._missionWaypoints.map((e) => Marker(markerId: MarkerId("wp_${e.latitude}_${e.longitude}"), position: e, icon: BitmapDescriptor.defaultMarkerWithHue(BitmapDescriptor.hueYellow)))
                                 },
                                 zoomControlsEnabled: false,
                                 myLocationButtonEnabled: false,
                               );
                             }
                           );
                         } catch (e) {
                           return Container(color: Colors.black87, child: const Center(child: Text("MAP ERROR", textAlign: TextAlign.center, style: TextStyle(color: Colors.white30))));
                         }
                       }),
                       
                       // COLLAPSE ICON
                       if (!_isMapExpanded) const Positioned(top: 8, right: 8, child: Icon(Icons.open_in_full, color: Colors.white70, size: 20)),
                       
                       // EXPANDED UI
                       if (_isMapExpanded) SafeArea(child: Padding(padding: const EdgeInsets.all(20), child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [Row(mainAxisAlignment: MainAxisAlignment.spaceBetween, children: [Container(padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6), decoration: BoxDecoration(color: Colors.black87, borderRadius: BorderRadius.circular(20), border: Border.all(color: neonTeal)), child: const Text("MISSION PLANNER", style: TextStyle(fontWeight: FontWeight.bold, color: Colors.white))), IconButton(onPressed: _toggleMap, icon: const Icon(Icons.close_fullscreen, color: Colors.white), style: IconButton.styleFrom(backgroundColor: Colors.black54))]), const Spacer(), Row(mainAxisAlignment: MainAxisAlignment.center, children: [FloatingActionButton.extended(onPressed: () => setState(() => _isMissionMode = !_isMissionMode), backgroundColor: _isMissionMode ? neonYellow : Colors.grey[800], label: Text(_isMissionMode ? "TAP TO ADD POINTS" : "CREATE PATH", style: TextStyle(color: _isMissionMode ? Colors.black : Colors.white)), icon: Icon(_isMissionMode ? Icons.touch_app : Icons.edit_road, color: _isMissionMode ? Colors.black : Colors.white)), const SizedBox(width: 20), if (_missionWaypoints.isNotEmpty) FloatingActionButton.extended(backgroundColor: neonTeal, onPressed: () { 
                           _toggleMap(); 
                           _addLog("UPLOADING MISSION..."); 
                           // Send Real Mission to Server
                           context.read<CentralState>().telemetry.sendMission(
                               _missionWaypoints.map((e) => {"lat": e.latitude, "lng": e.longitude}).toList() 
                           );
                           setState(() => flightMode = "AUTO-WP"); 
                       }, label: const Text("START"), icon: const Icon(Icons.play_arrow))])]))),
                     ],
                   ),
                 ),
               ),
             ),
           ),

                      
                      // AI RESPONSE TOAST (Restored "Fade Away Check")
                      Positioned(
                        top: 220, // Moved down to avoid Map overlap
                        left: 20, right: 20, // Padding
                        child: ValueListenableBuilder<String?>(
                          valueListenable: aiResponseVN,
                          builder: (context, msg, _) {
                            if (msg == null) return const SizedBox.shrink();
                            return Center(
                              child: Container(
                                height: 40, // THIN fixed height
                                width: double.infinity,
                                padding: const EdgeInsets.symmetric(horizontal: 16),
                                decoration: BoxDecoration(
                                  color: Colors.black.withOpacity(0.9),
                                  borderRadius: BorderRadius.circular(20),
                                  border: Border.all(color: const Color(0xFF2DD4BF), width: 1)
                                ),
                                alignment: Alignment.center,
                                child: SingleChildScrollView(
                                  scrollDirection: Axis.horizontal,
                                  child: Text(
                                    "> $msg", 
                                    style: const TextStyle(color: Color(0xFF2DD4BF), fontFamily: 'Courier', fontSize: 13, fontWeight: FontWeight.bold),
                                    maxLines: 1,
                                  )
                                )
                              )
                            );
                          }
                        )
                      ),



                      // Cam Controls - Moved to Left Center LOWER
                      // HIDDEN IN MISSION MODE (Map Full Screen)
                      if (!_isMapExpanded)
                        Positioned(
                          left: 20, 
                          top: size.height * 0.45, 
                          child: Column(
                            children: [
                               // Smaller Cam Buttons (40px)
                               SizedBox(width: 40, height: 40, child: CamButton(icon: Icons.camera, onTap: () {
                                  _addLog("IMG CAPTURED");
                                  context.read<CentralState>().telemetry.sendCommand("CAPTURE_PHOTO");
                               })),
                               const SizedBox(height: 15),
                               SizedBox(width: 40, height: 40, child: CamButton(
                                  icon: _isRecording ? Icons.stop : Icons.missed_video_call, 
                                  color: _isRecording ? Colors.red : const Color(0xFFF43F5E), 
                                  onTap: () {
                                    setState(() {
                                      _isRecording = !_isRecording;
                                      _addLog(_isRecording ? "REC STARTED" : "REC STOPPED");
                                      context.read<CentralState>().telemetry.sendCommand(_isRecording ? "START_RECORDING" : "STOP_RECORDING");
                                    });
                                  }
                               )),
                               // NEW ARM BUTTON (Below Record)
                               const SizedBox(height: 15),
                               SizedBox(width: 40, height: 40, child: CamButton(
                                  icon: Icons.shield,
                                  color: Colors.red,
                                  onTap: () {
                                    _addLog("ARMING DRONE!");
                                    context.read<CentralState>().telemetry.sendCommand("ARM");
                                    context.read<CentralState>().telemetry.sendControl(0, 0, -1.0, 0);
                                  }
                               )),
                               // DISARM BUTTON (below ARM, same size) — confirm popup -> force-disarm
                               const SizedBox(height: 15),
                               SizedBox(width: 40, height: 40, child: CamButton(
                                  icon: Icons.power_settings_new,
                                  color: Colors.orangeAccent,
                                  onTap: () => _confirmDisarm(context),
                               )),
                            ]
                          )
                        ),
                   ]
                 ),
               )
            ),

           // Main Interface (UI Controls) - NO LONGER FADES OUT, JUST IGNORES POINTER
           // This allows controls to sit ON TOP of the expanded map if needed, or at least not disappear.
           // However, if map is full screen, we might WANT them to disappear? 
           // User asked for "Fix Layering". Moving Map to index 2 (below) means controls are ON TOP.
           // So we keep opacity 1.0 but maybe IgnorePointer? 
           // If controls are on top, we can still use them. 
           // Let's REMOVE AnimatedOpacity to keep them visible.
           IgnorePointer(
               ignoring: _isMapExpanded, // Still ignore touches on controls if map is full screen? 
               // Actually, if we want to use the map, controls might get in the way.
               // But usually "Layering issue" means map was hiding controls or vice versa in a bad way.
               // If Map is expanded, it covers the screen. Controls on top is fine if they are minimized.
               // But the current controls are "Joysticks" which fill the screen bottom.
               // If Map is expanded, we probably DO want to hide joysticks.
               // So let's keep AnimatedOpacity but ensure Map is NOT inside it.
               child: AnimatedOpacity(
                 duration: const Duration(milliseconds: 300),
                 opacity: _isMapExpanded ? 0.0 : 1.0, 
                 child: Stack(
                 fit: StackFit.expand,
                 children: [
                   // Top Bar
                   Positioned(
                     top: 0, left: 0, right: 0,
                     child: SafeArea(
                       child: Padding(
                         padding: const EdgeInsets.symmetric(horizontal: 20, vertical: 10),
                         child: Row(
                           mainAxisAlignment: MainAxisAlignment.spaceBetween,
                           children: [
                              Row(children: [
                                IconButton(icon: const Icon(Icons.arrow_back, color: Colors.white70), onPressed: () => Navigator.of(context).pop()),
                                const SizedBox(width: 8),
                                const Icon(Icons.flight_takeoff, color: Colors.white70),
                                const SizedBox(width: 8),
                                Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
                                  Text(flightMode, style: TextStyle(color: neonTeal, fontWeight: FontWeight.bold, fontSize: 16)),
                                  const Text("READY TO FLY", style: TextStyle(fontSize: 10, color: Colors.white54)),
                                ])
                              ]),
                              // Telemetry Row
                              Consumer<CentralState>(
                                builder: (context, state, child) {
                                  final isImperial = state.config['units'] == 'imperial';
                                  String distUnit = isImperial ? "ft" : "m";
                                  String speedUnit = isImperial ? "mph" : "m/s";
                                  double distMult = isImperial ? 3.28084 : 1.0;
                                  double speedMult = isImperial ? 2.23694 : 1.0;

                                  return Row(mainAxisSize: MainAxisSize.min, children: [
                                      ValueListenableBuilder(valueListenable: altitudeVN, builder: (c,v,_) => TopTelemetryItem(icon: Icons.arrow_upward, value: (v * distMult).toStringAsFixed(1), unit: distUnit)),
                                      ValueListenableBuilder(valueListenable: speedVN, builder: (c,v,_) => TopTelemetryItem(icon: Icons.speed, value: (v * speedMult).toStringAsFixed(1), unit: speedUnit)),
                                      ValueListenableBuilder(valueListenable: distanceVN, builder: (c,v,_) => TopTelemetryItem(icon: Icons.straighten, value: (v * distMult).toStringAsFixed(0), unit: distUnit)),
                                  ]);
                                }
                              ),
                              // Status Icons
                              Row(children: [
                                // NETWORK TYPE (Real Binding)
                                ValueListenableBuilder(
                                  valueListenable: distanceVN, // Using signal strength logic 
                                  builder: (c,v,_) {
                                    // Hack: We don't have explicit network type in telemetry yet, 
                                    // but we can infer connection health.
                                    final bool connected = context.read<CentralState>().isDeviceConnected;
                                    return Row(children: [
                                      Icon(connected ? Icons.wifi : Icons.wifi_off, color: connected ? Colors.white : Colors.red, size: 16), 
                                      const SizedBox(width: 4), 
                                      Text(connected ? "4G/5G" : "DISC.", style: TextStyle(color: connected ? Colors.white : Colors.red, fontSize: 12))
                                    ]);
                                  }
                                ),
                                const SizedBox(width: 15),
                                
                                // SATELLITES (Real Binding)
                                ValueListenableBuilder<int>(
                                  valueListenable: satsVN, 
                                  builder: (c,v,_) => Row(children: [
                                    Icon(Icons.satellite_alt, color: v > 6 ? Colors.green : (v > 0 ? neonYellow : Colors.grey), size: 16), 
                                    const SizedBox(width: 4), 
                                    Text("$v Sats", style: TextStyle(color: v > 6 ? Colors.green : (v > 0 ? neonYellow : Colors.grey), fontSize: 12))
                                  ])
                                ),
                                const SizedBox(width: 15),
                                
                                // BATTERY (Real Binding)
                                ValueListenableBuilder<double>(
                                  valueListenable: batteryVN, 
                                  builder: (c, v, _) => Row(children: [
                                    Icon(v < 20 ? Icons.battery_alert : Icons.battery_std, color: v < 20 ? neonRed : Colors.green, size: 16), 
                                    const SizedBox(width: 4), 
                                    Text("${v.toInt()}%", style: TextStyle(color: v < 20 ? neonRed : Colors.green, fontSize: 12))
                                  ])
                                ),
                                
                                IconButton(icon: const Icon(Icons.settings, color: Colors.white), onPressed: () => setState(() { _showSettingsSidebar = !_showSettingsSidebar; _showAnalyticsSidebar = false; }))
                              ])
                           ],
                         ),
                       ),
                     ),
                   ),
                   // Left Logs
                   Positioned(
                     top: 50, left: 20,
                     child: RepaintBoundary( // Logs change frequently, isolate repaint
                       child: Container(
                         width: 200, height: 80,
                         margin: const EdgeInsets.only(bottom: 20),
                         padding: const EdgeInsets.all(8),
                         decoration: BoxDecoration(color: Colors.black45, border: Border(left: BorderSide(color: neonTeal, width: 2))),
                         child: ValueListenableBuilder(
                           valueListenable: logsVN,
                           builder: (c, val, _) => SingleChildScrollView(reverse: true, child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: val.map((l) => Text(l, style: TextStyle(fontSize: 10, color: neonTeal.withOpacity(0.8)))).toList())),
                         ),
                       ),
                     ),
                   ),
                   // Bottom Controls
                   Positioned(
                     bottom: 20, left: size.width * 0.05, right: size.width * 0.05,
                     child: Row(
                       mainAxisAlignment: MainAxisAlignment.spaceBetween,
                       crossAxisAlignment: CrossAxisAlignment.end,
                       children: [
                         RepaintBoundary(
                           child: VirtualJoystick(
                             size: joystickSize, 
                             color: neonTeal, 
                             onMove: (v) {
                               setState(() => leftStick = v);
                               if (context.mounted) {
                                  // P1.5: Apply Sensitivity
                                  final sens = (context.read<CentralState>().config['sensitivity'] ?? 0.7).toDouble();
                                  context.read<CentralState>().telemetry.sendControl(0, 0, -v.dy * sens, v.dx * sens);
                               }
                             }
                           ),
                         ),
                         
                         // Middle Buttons
                         Expanded(
                            child: Padding(
                              padding: const EdgeInsets.only(bottom: 20),
                              child: Row(
                                mainAxisAlignment: MainAxisAlignment.spaceEvenly, 
                                children: [
                                  _DashboardBtn(size: btnSize, icon: Icons.flight_land, label: "LAND", onTap: () {
                                     _addLog("LANDING...");
                                     context.read<CentralState>().telemetry.sendCommand("LAND");
                                  }),
                                  _DashboardBtn(size: btnSize, icon: Icons.analytics, label: "STATS", color: _showAnalyticsSidebar ? neonBlue : Colors.white70, onTap: () => setState(() { _showAnalyticsSidebar = !_showAnalyticsSidebar; _showSettingsSidebar = false; })),
                                  GestureDetector(onTap: () => _showAIModal(context), child: _HeroAIButton(size: btnSize * 1.2, color: neonTeal)),
                                  _DashboardBtn(size: btnSize, icon: Icons.home, label: "RTH", onTap: () {
                                     _addLog("RTH TRIGGERED");
                                     context.read<CentralState>().telemetry.sendCommand("RTH");
                                  }),
                                ],
                              ),
                            ),
                          ),

                          
                          // Right Joystick
                          RepaintBoundary(
                            child: VirtualJoystick(
                              size: joystickSize, 
                              color: neonTeal, 
                              onMove: (v) {
                                setState(() => rightStick = v);
                                // MA-1 FIX: Apply sensitivity multiplier (left stick has it, right was missing)
                                final sens = (context.read<CentralState>().config['sensitivity'] ?? 0.7).toDouble();
                                context.read<CentralState>().telemetry.sendControl(v.dx * sens, -v.dy * sens, 0, 0);
                              }
                            ),
                          ),
                       ],
                     ),
                   ),
                 ],
               ),
             ),
           ),
            
           // SIDEBARS
           if (_showSettingsSidebar)
              Positioned(
                right: 0, top: 0, bottom: 0, 
                child: SettingsSidebar(
                  onClose: () => setState(() => _showSettingsSidebar = false),
                )
              ),
              
           if (_showAnalyticsSidebar)
              Positioned(
                right: 0, top: 0, bottom: 0, 
                child: Consumer<CentralState>( // Use Consumer to get latest history updates
                  builder: (context, state, child) => AnalyticsSidebar(
                     onClose: () => setState(() => _showAnalyticsSidebar = false),
                     altHistory: state.altHistory,
                     speedHistory: state.speedHistory,
                     batHistory: [], // Add if tracked
                     currentAlt: altitudeVN.value,
                     currentSpeed: speedVN.value,
                     currentDist: distanceVN.value,
                     currentRoll: rollVN.value,
                     currentPitch: pitchVN.value,
                     currentYaw: yawVN.value,
                  ),
                )
              ),

            ],
          );
        },
      ),
    );
  }
}

// Helpers
class _DashboardBtn extends StatelessWidget {
  final IconData icon;
  final String label;
  final VoidCallback onTap;
  final Color color;
  final double size; // NEW: Dynamic Size
  const _DashboardBtn({required this.icon, required this.label, required this.onTap, this.color = Colors.white70, required this.size});
  @override
  Widget build(BuildContext context) {
    return GestureDetector(onTap: onTap, child: Column(children: [
      GlassContainer(borderRadius: 16, child: Container(
        width: size, height: size,
        alignment: Alignment.center,
        child: Icon(icon, color: color, size: size * 0.5), // Sensitive Icon Scaling
      )), 
      const SizedBox(height: 4), 
      Text(label, style: TextStyle(fontSize: size * 0.18, color: color.withOpacity(0.7), fontWeight: FontWeight.bold)) // Scaled Text
    ]));
  }
}

class _HeroAIButton extends StatelessWidget {
  final Color color;
  final double size; 
  const _HeroAIButton({super.key, required this.color, required this.size});
  @override
  Widget build(BuildContext context) {
    return Column(children: [
      Container(
        height: size, width: size, 
        decoration: BoxDecoration(
          color: color, 
          borderRadius: BorderRadius.circular(size * 0.35), 
          boxShadow: [BoxShadow(color: color.withOpacity(0.4), blurRadius: 20)]
        ), 
        child: Icon(Icons.psychology, color: Colors.black, size: size * 0.55)
      ),
      const SizedBox(height: 5),
      Container(
        padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 2), 
        decoration: BoxDecoration(color: Colors.black54, borderRadius: BorderRadius.circular(4)), 
        child: Text("AI PILOT", style: TextStyle(color: color, fontSize: size * 0.18, fontWeight: FontWeight.bold))
      )
    ]);
  }
}

class TopTelemetryItem extends StatelessWidget {
  final String value, unit;
  final IconData icon;
  const TopTelemetryItem({super.key, required this.value, required this.unit, required this.icon});
  @override
  Widget build(BuildContext context) {
    return Padding(padding: const EdgeInsets.symmetric(horizontal: 10), child: Row(children: [Icon(icon, size: 16, color: Colors.white70), const SizedBox(width: 5), Text(value, style: const TextStyle(fontSize: 16, fontWeight: FontWeight.bold, color: Colors.white)), const SizedBox(width: 2), Text(unit, style: const TextStyle(fontSize: 10, color: Colors.white54))]));
  }
}

class _AIChip extends StatelessWidget {
  final String label;
  final Color color;
  final VoidCallback onTap;
  const _AIChip({required this.label, required this.color, required this.onTap});
  @override
  Widget build(BuildContext context) {
    return Padding(padding: const EdgeInsets.only(right: 8.0), child: ActionChip(label: Text(label, style: TextStyle(color: color, fontSize: 12)), backgroundColor: Colors.white10, side: BorderSide(color: color.withOpacity(0.3)), onPressed: onTap));
  }
}

class CamButton extends StatelessWidget {
  final IconData icon;
  final VoidCallback onTap;
  final Color color;
  const CamButton({super.key, required this.icon, required this.onTap, this.color = Colors.white});
  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onTap: onTap,
      child: Container(
        width: 40, height: 40,
        alignment: Alignment.center,
        decoration: BoxDecoration(
          shape: BoxShape.circle, 
          color: Colors.white10, 
          border: Border.all(color: Colors.white24)
        ), 
        child: Center(child: Icon(icon, color: color, size: 20))
      )
    );
  }
}
