import 'dart:ui';
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../services/central_state.dart';
import '../widgets/shared_ui.dart';
import 'signup_screen.dart';

class LoginScreen extends StatefulWidget {
  const LoginScreen({super.key});

  @override
  State<LoginScreen> createState() => _LoginScreenState();
}

class _LoginScreenState extends State<LoginScreen> {
  final _usernameCtrl = TextEditingController();
  final _passwordCtrl = TextEditingController();

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      resizeToAvoidBottomInset: true, 
      body: Stack(
        children: [
          Image.network(
            'https://images.unsplash.com/photo-1579829366248-204fe8413f31?q=80&w=2070&auto=format&fit=crop',
            fit: BoxFit.cover,
            width: double.infinity,
            height: double.infinity,
          ),
          BackdropFilter(
            filter: ImageFilter.blur(sigmaX: 10, sigmaY: 10),
            child: Container(color: Colors.black.withOpacity(0.5)),
          ),
          Center(
            child: SingleChildScrollView(
              padding: const EdgeInsets.all(20),
              child: GlassContainer(
                color: Colors.black.withOpacity(0.7),
                borderColor: const Color(0xFF2DD4BF),
                child: Container(
                  width: 450,
                  padding: const EdgeInsets.all(40),
                  child: Column(
                    mainAxisSize: MainAxisSize.min,
                    children: [
                      const Icon(Icons.person_pin_circle, size: 60, color: Color(0xFF2DD4BF)),
                      const SizedBox(height: 20),
                      const Text("PILOT LOGIN", style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold, letterSpacing: 2, color: Colors.white)),
                      const SizedBox(height: 40),
                       
                      TextField(
                        controller: _usernameCtrl,
                        style: const TextStyle(color: Colors.white),
                        decoration: InputDecoration(
                          labelText: "USERNAME / EMAIL", 
                          prefixIcon: const Icon(Icons.person, color: Colors.white70),
                          filled: true,
                          fillColor: Colors.white10,
                          border: OutlineInputBorder(borderRadius: BorderRadius.circular(10), borderSide: BorderSide.none),
                          labelStyle: const TextStyle(color: Colors.white70),
                        ),
                      ),
                      const SizedBox(height: 20),
                       
                      TextField(
                        controller: _passwordCtrl,
                        obscureText: true,
                        style: const TextStyle(color: Colors.white),
                        decoration: InputDecoration(
                          labelText: "PASSWORD",
                          prefixIcon: const Icon(Icons.lock, color: Colors.white70),
                          filled: true,
                          fillColor: Colors.white10,
                          border: OutlineInputBorder(borderRadius: BorderRadius.circular(10), borderSide: BorderSide.none),
                          labelStyle: const TextStyle(color: Colors.white70),
                        ),
                      ),
                      const SizedBox(height: 40),
                      SizedBox(
                        width: double.infinity,
                        height: 50,
                        child: ElevatedButton(
                          onPressed: () async {
                             final success = await context.read<CentralState>().login(
                               _usernameCtrl.text, 
                               // Real password input
                               _passwordCtrl.text 
                             );
                             
                             if (mounted) {
                               if (success) {
                                 Navigator.of(context).pushNamedAndRemoveUntil('/connect', (route) => false);
                               } else {
                                 ScaffoldMessenger.of(context).showSnackBar(
                                   const SnackBar(content: Text("LOGIN OPTICAL LINK FAILED. RETRY."), backgroundColor: Colors.red)
                                 );
                               }
                             }
                          },
                          style: ElevatedButton.styleFrom(backgroundColor: const Color(0xFF2DD4BF), foregroundColor: Colors.black),
                          child: const Text("CONTINUE", style: TextStyle(fontWeight: FontWeight.bold)),
                        ),
                      ),
                      const SizedBox(height: 20),
                      TextButton(
                        onPressed: () {
                          Navigator.push(context, MaterialPageRoute(builder: (c) => SignupScreen(
                             onSignupSuccess: () => Navigator.pop(context),
                             onLoginPress: () => Navigator.pop(context),
                          )));
                        },
                        child: const Text("Create Account", style: TextStyle(color: Colors.white70)),
                      )
                    ],
                  ),
                ),
              ),
            ),
          )
        ],
      ),
    );
  }
}
