import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'package:path_provider/path_provider.dart';

class FlightRecorderService {
  static final FlightRecorderService _instance = FlightRecorderService._internal();
  factory FlightRecorderService() => _instance;
  FlightRecorderService._internal();

  File? _currentLogFile;
  bool _isRecording = false;
  Timer? _recordTimer;
  
  // Stats
  DateTime? _startTime;
  double _maxAlt = 0;
  double _maxSpeed = 0;
  double _totalDist = 0;
  
  Future<void> startFlight() async {
     try {
       final dir = await getApplicationDocumentsDirectory();
       final logsDir = Directory('${dir.path}/flight_logs');
       if (!await logsDir.exists()) await logsDir.create(recursive: true);
       
       final timestamp = DateTime.now().millisecondsSinceEpoch;
       _currentLogFile = File('${logsDir.path}/flight_$timestamp.jsonl'); // JSON Lines
       
       _startTime = DateTime.now();
       _maxAlt = 0;
       _maxSpeed = 0;
       _totalDist = 0;
       _isRecording = true;
       
       print("🔴 Flight Recorder Started: ${_currentLogFile!.path}");
     } catch (e) {
       print("Recorder Init Error: $e");
     }
  }

  Future<void> logFrame(Map<String, dynamic> telemetry, double lat, double lng) async {
     if (!_isRecording || _currentLogFile == null) return;
     
     try {
        final t = DateTime.now().millisecondsSinceEpoch;
        final alt = (telemetry['altitude'] ?? 0).toDouble();
        final speed = (telemetry['speed'] ?? 0).toDouble();
        
        // Update Stats
        if (alt > _maxAlt) _maxAlt = alt;
        if (speed > _maxSpeed) _maxSpeed = speed;
        
        final frame = {
           "t": t,
           "lat": lat,
           "lng": lng,
           "alt": alt,
           "spd": speed,
           "bat": telemetry['battery'] ?? 0
        };
        
        await _currentLogFile!.writeAsString(jsonEncode(frame) + "\n", mode: FileMode.append);
     } catch (e) {
        // print("Log Write Error");
     }
  }

  Future<void> stopFlight() async {
     if (!_isRecording) return;
     _isRecording = false;
     
     // Write Summary at end (or a separate index file)
     final duration = DateTime.now().difference(_startTime!).inSeconds;
     print("🏁 Flight Ended. Duration: ${duration}s. Max Alt: $_maxAlt");
     
     // Update Fleet Stats (REAL PERSISTENCE)
     try {
        final dir = await getApplicationDocumentsDirectory();
        final statsFile = File('${dir.path}/fleet_stats.json');
        
        Map<String, dynamic> stats = {"total_airtime": 0, "total_distance": 0, "total_missions": 0};
        
        if (await statsFile.exists()) {
           stats = jsonDecode(await statsFile.readAsString());
        }
        
        stats["total_airtime"] = (stats["total_airtime"] as num) + duration;
        stats["total_distance"] = (stats["total_distance"] as num) + _totalDist;
        stats["total_missions"] = (stats["total_missions"] as num) + 1;
        
        await statsFile.writeAsString(jsonEncode(stats));
        print("✅ Fleet Stats Updated: $stats");
     } catch (e) {
        print("Error saving fleet stats: $e");
     }
  }
  
  // --- READERS (For Hangar UI) ---
  
  Future<Map<String, String>> getFleetStats() async {
    try {
       final dir = await getApplicationDocumentsDirectory();
       final statsFile = File('${dir.path}/fleet_stats.json');
       if (await statsFile.exists()) {
          final stats = jsonDecode(await statsFile.readAsString());
          final distKm = ((stats['total_distance'] ?? 0) / 1000.0).toStringAsFixed(1);
          final hours = ((stats['total_airtime'] ?? 0) / 3600.0).floor();
          final mins = (((stats['total_airtime'] ?? 0) % 3600) / 60).floor();
          
          return {
             "airtime": "${hours}h ${mins}m",
             "distance": "$distKm km",
             "missions": "${stats['total_missions']} Sorties"
          };
       }
    } catch (e) { print("Stats Read Error: $e"); }
    
    return {"airtime": "0h 0m", "distance": "0.0 km", "missions": "0 Sorties"}; 
  }

  Future<List<Map<String, dynamic>>> getPastFlights() async {
     List<Map<String, dynamic>> flights = [];
     try {
       final dir = await getApplicationDocumentsDirectory();
       final logsDir = Directory('${dir.path}/flight_logs');
       if (await logsDir.exists()) {
          final files = logsDir.listSync().whereType<File>().where((f) => f.path.endsWith('.jsonl')).toList();
          // Sort new to old
          files.sort((a,b) => b.lastModifiedSync().compareTo(a.lastModifiedSync()));
          
          for (var f in files) {
             final sizeCb = (f.lengthSync() / 1024).toStringAsFixed(0) + " KB";
             final dateStr = f.lastModifiedSync().toString().split('.')[0];
             flights.add({
                "date": dateStr,
                "size": sizeCb,
                "path": f.path
             });
          }
       }
     } catch (e) { print("Logs Read Error: $e"); }
     return flights;
  }
  
  // No dummy data allowed by strict user policy.
  // Init logic handles directory creation.
}

