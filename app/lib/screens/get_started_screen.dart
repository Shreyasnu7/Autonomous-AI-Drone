import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'login_screen.dart';

class GetStartedScreen extends StatelessWidget {
  const GetStartedScreen({super.key});

  @override
  Widget build(BuildContext context) {
    // Ensure landscape for this screen too as per user code
    SystemChrome.setPreferredOrientations([
      DeviceOrientation.landscapeLeft,
      DeviceOrientation.landscapeRight,
    ]);

    return Scaffold(
      body: Stack(
        fit: StackFit.expand,
        children: [
          Image.network(
            'https://images.unsplash.com/photo-1506947411487-a56738267384?q=80&w=2070&auto=format&fit=crop',
            fit: BoxFit.cover,
            errorBuilder: (c, e, s) => Container(color: Colors.black),
          ),
          Container(
            decoration: BoxDecoration(
              gradient: LinearGradient(
                begin: Alignment.topCenter,
                end: Alignment.bottomCenter,
                colors: [Colors.transparent, Colors.black.withOpacity(0.8), Colors.black],
                stops: const [0.5, 0.8, 1.0],
              ),
            ),
          ),
          Padding(
            padding: const EdgeInsets.all(40.0),
            child: Column(
              mainAxisAlignment: MainAxisAlignment.end,
              crossAxisAlignment: CrossAxisAlignment.center,
              children: [
                const Text("NEON LINK", style: TextStyle(fontSize: 42, fontWeight: FontWeight.bold, letterSpacing: 3, color: Colors.white)),
                const SizedBox(height: 10),
                const Text("The ultimate telemetry & control system for your UAV.", style: TextStyle(color: Colors.white70, fontSize: 14)),
                const SizedBox(height: 50),
                SizedBox(
                  width: 300,
                  height: 55,
                  child: ElevatedButton(
                    onPressed: () {
                      Navigator.push(context, MaterialPageRoute(builder: (c) => const LoginScreen()));
                    },
                    style: ElevatedButton.styleFrom(
                      backgroundColor: const Color(0xFF2DD4BF), 
                      foregroundColor: Colors.black,
                      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(30)),
                    ),
                    child: const Text("START THE JOURNEY", style: TextStyle(fontWeight: FontWeight.bold, fontSize: 16)),
                  ),
                ),
                const SizedBox(height: 20),
              ],
            ),
          )
        ],
      ),
    );
  }
}
