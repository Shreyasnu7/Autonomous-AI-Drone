import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';

import 'theme/app_theme.dart';
import 'services/central_state.dart';
import 'services/api_service.dart';
import 'screens/get_started_screen.dart';
import 'screens/login_screen.dart';
import 'screens/connect_device_screen.dart';
import 'screens/hangar_screen.dart';
import 'screens/control_screen.dart';
import 'screens/gallery_screen.dart';
import 'screens/signup_screen.dart'; // Ensure exported

void main() async {
  WidgetsFlutterBinding.ensureInitialized();
  
  // Set orientation landscape as per user requirement
  await SystemChrome.setPreferredOrientations([
    DeviceOrientation.landscapeLeft,
    DeviceOrientation.landscapeRight,
  ]);
  
  // System UI mode
  SystemChrome.setEnabledSystemUIMode(SystemUiMode.immersiveSticky);

  final state = CentralState();
  await state.init();
  ApiService.wakeUp(); // Fire and forget to wake server
  
  runApp(
    ChangeNotifierProvider.value(
      value: state,
      child: const NeonDroneApp(),
    ),
  );
}

class NeonDroneApp extends StatelessWidget {
  const NeonDroneApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Neon Drone Controller',
      debugShowCheckedModeBanner: false,
      theme: ThemeData.dark().copyWith(
        scaffoldBackgroundColor: const Color(0xFF000000),
        textTheme: ThemeData.dark().textTheme.apply(
              fontFamily: 'Courier', // Matches user font choice
              bodyColor: Colors.white,
            ),
        useMaterial3: false, 
      ),
      home: const FlowDispatcher(),
      routes: {
        '/get_started': (ctx) => const GetStartedScreen(),
        '/login': (ctx) => const LoginScreen(),
        '/connect': (ctx) => const ConnectScreen(),
        '/hangar': (ctx) => const HangarScreen(),
        '/control': (ctx) => const ControlScreen(),
        '/gallery': (ctx) => const GalleryScreen(),
        // Signup is usually pushed via MaterialPageRoute to allow easy back navigation, 
        // but we can register it if needed
      },
    );
  }
}

class FlowDispatcher extends StatelessWidget {
  const FlowDispatcher({super.key});

  @override
  Widget build(BuildContext context) {
    final state = context.watch<CentralState>();
    
    // Logic: 
    // 1. If not logged in -> Get Started (which leads to Login)
    // 2. If logged in but no device -> Connect Screen
    // 3. If everything ready -> Hangar
    
    if (state.isLoggedIn && state.connectedDroneId != null) {
      return const HangarScreen();
    } else if (state.isLoggedIn) {
      return const ConnectScreen();
    } else {
      return const GetStartedScreen();
    }
  }
}
