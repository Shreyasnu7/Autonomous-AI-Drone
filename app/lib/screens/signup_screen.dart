import 'package:flutter/material.dart';
import 'package:animate_do/animate_do.dart';
import 'dart:ui';
import 'package:provider/provider.dart';
import '../services/auth_service.dart';
import '../services/central_state.dart';

class SignupScreen extends StatefulWidget {
  final VoidCallback onSignupSuccess;
  final VoidCallback onLoginPress;

  const SignupScreen({
    super.key,
    required this.onSignupSuccess,
    required this.onLoginPress,
  });

  @override
  State<SignupScreen> createState() => _SignupScreenState();
}

class _SignupScreenState extends State<SignupScreen> {
  final _formKey = GlobalKey<FormState>();
  final _emailCtrl = TextEditingController();
  final _passCtrl = TextEditingController();
  final _confirmPassCtrl = TextEditingController();
  final _nameCtrl = TextEditingController();

  Future<void> _handleSignup() async {
    if (_formKey.currentState!.validate()) {
      if (_passCtrl.text != _confirmPassCtrl.text) {
         ScaffoldMessenger.of(context).showSnackBar(const SnackBar(content: Text("SECURITY KEYS DO NOT MATCH"), backgroundColor: Colors.red));
         return;
      }
      
      try {
        await AuthService.signup(_nameCtrl.text, _emailCtrl.text, _passCtrl.text);
        widget.onSignupSuccess();
      } catch (e) {
        ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text("REGISTRATION ERROR: $e"), backgroundColor: Colors.red));
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    const neonTeal = Color(0xFF2DD4BF);
    const neonPink = Color(0xFFF43F5E);
    
    return Scaffold(
      backgroundColor: Colors.black,
      body: Stack(
        fit: StackFit.expand,
        children: [
          // Background Image with Blur
          Image.network(
            'https://images.unsplash.com/photo-1579829366248-204fe8413f31?q=80&w=2070&auto=format&fit=crop', // Cyberpunk City
            fit: BoxFit.cover,
            errorBuilder: (c,e,s) => Container(color: const Color(0xFF111111)),
          ),
          BackdropFilter(
            filter: ImageFilter.blur(sigmaX: 10, sigmaY: 10),
            child: Container(color: Colors.black.withOpacity(0.7)), // Dark Overlay
          ),

          // Content
          Center(
            child: SingleChildScrollView(
              padding: const EdgeInsets.all(24),
              child: FadeInUp(
                duration: const Duration(milliseconds: 600),
                child: ClipRRect(
                  borderRadius: BorderRadius.circular(20),
                  child: BackdropFilter(
                    filter: ImageFilter.blur(sigmaX: 10, sigmaY: 10),
                    child: Container(
                      padding: const EdgeInsets.all(40),
                      constraints: const BoxConstraints(maxWidth: 400),
                      decoration: BoxDecoration(
                        color: Colors.white.withOpacity(0.05),
                        borderRadius: BorderRadius.circular(20),
                        border: Border.all(color: Colors.white12),
                        boxShadow: [BoxShadow(color: neonTeal.withOpacity(0.1), blurRadius: 30)]
                      ),
                      child: Form(
                        key: _formKey,
                        child: Column(
                          mainAxisSize: MainAxisSize.min,
                          children: [
                            // Logo / Icon
                            Icon(Icons.person_add_alt_1, size: 50, color: neonTeal),
                            const SizedBox(height: 20),
                            Text(
                              "JOIN THE FLEET",
                              style: TextStyle(
                                fontSize: 24,
                                fontWeight: FontWeight.bold,
                                color: Colors.white,
                                letterSpacing: 2,
                                shadows: [Shadow(color: neonTeal, blurRadius: 10)]
                              ),
                            ),
                            const SizedBox(height: 10),
                            const Text("Create your pilot credentials", style: TextStyle(color: Colors.white54, fontSize: 12)),
                            
                            const SizedBox(height: 40),
                            
                            // Fields
                            _GlassTextField(controller: _nameCtrl, icon: Icons.badge, hint: "CALLSIGN / NAME", color: neonTeal),
                            const SizedBox(height: 15),
                            _GlassTextField(controller: _emailCtrl, icon: Icons.email, hint: "EMAIL COMMS", color: neonTeal),
                            const SizedBox(height: 15),
                            _GlassTextField(controller: _passCtrl, icon: Icons.lock, hint: "SECURITY KEY", obscure: true, color: neonPink),
                            const SizedBox(height: 15),
                            _GlassTextField(controller: _confirmPassCtrl, icon: Icons.lock_clock, hint: "CONFIRM KEY", obscure: true, color: neonPink),

                            const SizedBox(height: 40),
                            
                            // Button
                            SizedBox(
                              width: double.infinity,
                              height: 50,
                              child: ElevatedButton(
                                style: ElevatedButton.styleFrom(
                                  backgroundColor: neonTeal,
                                  foregroundColor: Colors.black,
                                  shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(10)),
                                  elevation: 10,
                                  shadowColor: neonTeal.withOpacity(0.5),
                                ),
                                onPressed: _handleSignup,
                                child: const Text("INITIALIZE LINK", style: TextStyle(fontWeight: FontWeight.bold, letterSpacing: 1.5)),
                              ),
                            ),
                            
                            const SizedBox(height: 20),
                            TextButton(
                              onPressed: widget.onLoginPress,
                              child: RichText(
                                text: TextSpan(
                                  style: const TextStyle(color: Colors.white54, fontSize: 12),
                                  children: [
                                    const TextSpan(text: "ALREADY ACTIVE? "),
                                    TextSpan(text: "LOGIN", style: TextStyle(color: neonTeal, fontWeight: FontWeight.bold))
                                  ]
                                )
                              ),
                            ),
                          ],
                        ),
                      ),
                    ),
                  ),
                ),
              ),
            ),
          ),
        ],
      ),
    );
  }
}

class _GlassTextField extends StatelessWidget {
  final TextEditingController controller;
  final IconData icon;
  final String hint;
  final bool obscure;
  final Color color;

  const _GlassTextField({required this.controller, required this.icon, required this.hint, this.obscure = false, required this.color});

  @override
  Widget build(BuildContext context) {
    return TextFormField(
      controller: controller,
      obscureText: obscure,
      style: const TextStyle(color: Colors.white, fontWeight: FontWeight.bold),
      cursorColor: color,
      decoration: InputDecoration(
        prefixIcon: Icon(icon, color: color.withOpacity(0.7), size: 18),
        hintText: hint,
        hintStyle: const TextStyle(color: Colors.white24, fontSize: 12, letterSpacing: 1),
        filled: true,
        fillColor: Colors.black45,
        border: OutlineInputBorder(borderRadius: BorderRadius.circular(10), borderSide: BorderSide.none),
        enabledBorder: OutlineInputBorder(borderRadius: BorderRadius.circular(10), borderSide: const BorderSide(color: Colors.white10)),
        focusedBorder: OutlineInputBorder(borderRadius: BorderRadius.circular(10), borderSide: BorderSide(color: color.withOpacity(0.5))),
      ),
      validator: (v) => v!.isEmpty ? "REQUIRED" : null,
    );
  }
}
