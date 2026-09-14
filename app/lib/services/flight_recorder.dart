import 'dart:convert';
import 'dart:io';
import 'package:path_provider/path_provider.dart';
import 'api_service.dart';

class FlightRecorderService {
  File? _currentLogFile;
  final List<Map<String, dynamic>> _buffer = [];
  bool isRecording = false;

  // --- RECORDING (LOCAL BUFFER -> UPLOAD LATER OPTION) ---
  
  Future<void> startRecording() async {
    isRecording = true;
    _buffer.clear();
    final dir = await getApplicationDocumentsDirectory();
    final timestamp = DateTime.now().toIso8601String().replaceAll(':', '-');
    _currentLogFile = File('${dir.path}/flight_log_$timestamp.json');
  }

  void addEntry(Map<String, dynamic> telemetry) {
    if (!isRecording) return;
    _buffer.add({
      "t": DateTime.now().millisecondsSinceEpoch,
      ...telemetry
    });
    
    if (_buffer.length >= 50) _flush();
  }

  Future<void> stopRecording() async {
    isRecording = false;
    await _flush();
    _currentLogFile = null;
    
    // Upload to Server
    try {
       // Calculate Stats
       double maxAlt = 0;
       double maxSpd = 0;
       double startBat = 0;
       double endBat = 0;
       
       if (_buffer.isNotEmpty) {
          startBat = (_buffer.first['battery'] as num).toDouble();
          endBat = (_buffer.last['battery'] as num).toDouble();
          for (var e in _buffer) {
             // MA-14 FIX: telemetry uses 'altitude' and 'speed', not 'alt'/'spd'
             double a = (e['altitude'] ?? e['alt'] ?? 0 as num).toDouble();
             double s = (e['speed'] ?? e['spd'] ?? 0 as num).toDouble();
             if (a > maxAlt) maxAlt = a;
             if (s > maxSpd) maxSpd = s;
          }
       }
       
       // Create Payload
       final logData = {
          "date": DateTime.now().toIso8601String().split('T')[0],
          "duration": "${(_buffer.length * 0.2).toStringAsFixed(1)}s", // Approx 5hz
          "max_alt": maxAlt,
          "max_speed": maxSpd,
          "battery_used": startBat - endBat,
          "location": _buffer.isNotEmpty ? "${_buffer.first['lat']}, ${_buffer.first['lng']}" : "Unknown"
       };
       
       await ApiService.post('/logs', logData);
       print("Log Uploaded Successfully");
    } catch (e) {
       print("Log Upload Failed (Saved Locally Only): $e");
    }
  }

  Future<void> _flush() async {
    if (_currentLogFile == null || _buffer.isEmpty) return;
    final sink = _currentLogFile!.openWrite(mode: FileMode.append);
    for (var entry in _buffer) {
      sink.writeln(jsonEncode(entry));
    }
    await sink.flush();
    await sink.close();
    _buffer.clear();
  }

  // --- RETRIEVAL (FROM REAL SERVER DB) ---

  Future<List<Map<String, dynamic>>> getPastFlights() async {
    List<Map<String, dynamic>> serverLogs = [];
    try {
       // 1. Try Server
       final List<dynamic> data = await ApiService.get('/logs');
       serverLogs = data.map((e) => Map<String, dynamic>.from(e)).toList();
    } catch (e) {
       print("Server Log Fetch Failed (Using Local): $e");
    }

    // 2. Fetch Local Files (Always, or merge?)
    // For now, if server fails, use local.
    if (serverLogs.isEmpty) {
        try {
            final dir = await getApplicationDocumentsDirectory();
            final files = dir.listSync().where((f) => f.path.contains('flight_log_')).toList();
            
            List<Map<String, dynamic>> localLogs = [];
            for (var f in files) {
                if (f is File) {
                    // We only need summaries, but local files are raw streams.
                    // We quickly parse stats.
                    try {
                        final lines = await f.readAsLines();
                        if (lines.isNotEmpty) {
                           final first = jsonDecode(lines.first);
                           final last = jsonDecode(lines.last);
                           localLogs.add({
                               "date": (first['t'] != null) ? DateTime.fromMillisecondsSinceEpoch(first['t']).toIso8601String() : "Unknown",
                               "duration": "${(lines.length * 0.2).toStringAsFixed(1)}s",
                               "location": "Local Log",
                               "battery_used": 0.0, // Calc if need
                               "source": "local"
                           });
                        }
                    } catch (_) {}
                }
            }
            return localLogs.reversed.toList();
        } catch (e) {
            print("Local Log Fetch Error: $e");
        }
    }
    return serverLogs;
  }
  
  Future<Map<String, String>> getFleetStats() async {
    try {
        final logs = await getPastFlights();
        double totalSeconds = 0;
        double totalMaxAlt = 0;
        double totalMaxSpeed = 0;

        for (var l in logs) {
           // V110: Parse real duration from logs
           String durStr = (l['duration'] ?? "0s").toString();
           // Parse "123.4s" format
           if (durStr.endsWith('s')) {
              totalSeconds += double.tryParse(durStr.replaceAll('s', '')) ?? 0;
           }
           // Track max values
           double alt = (l['max_alt'] as num?)?.toDouble() ?? 0;
           double spd = (l['max_speed'] as num?)?.toDouble() ?? 0;
           if (alt > totalMaxAlt) totalMaxAlt = alt;
           if (spd > totalMaxSpeed) totalMaxSpeed = spd;
        }

        // Format airtime
        int totalMin = (totalSeconds / 60).floor();
        int hours = totalMin ~/ 60;
        int mins = totalMin % 60;
        String airtime = hours > 0 ? "${hours}h ${mins}m" : "${mins}m";

        return {
           "airtime": airtime,
           "distance": "${totalMaxAlt.toStringAsFixed(0)}m max alt",
           "missions": "${logs.length} Sorties"
        };
    } catch (_) {
       return {"airtime": "0m", "distance": "0m", "missions": "0"};
    }
  }
}
