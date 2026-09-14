import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:shared_preferences/shared_preferences.dart';
import '../services/central_state.dart';
import '../services/api_service.dart';
import '../theme/app_theme.dart';

class SettingsSidebar extends StatelessWidget {
  final VoidCallback onClose;
  const SettingsSidebar({super.key, required this.onClose});

  @override
  Widget build(BuildContext context) {
    final state = context.watch<CentralState>();
    final config = state.config;
    
    // Helper handlers with Sync
    void set(String k, dynamic v, {bool sync = true}) {
      context.read<CentralState>().updateConfig(k, v);
    }

    return Container(
      width: 350,
      height: double.infinity,
      decoration: BoxDecoration(
        color: const Color(0xFF111111).withOpacity(0.95),
        border: const Border(left: BorderSide(color: Color(0xFF2DD4BF), width: 1)),
        boxShadow: const [BoxShadow(color: Colors.black, blurRadius: 20)],
      ),
      child: DefaultTabController(
        length: 6,
        child: Column(
          children: [
            // Header
            Padding(
              padding: const EdgeInsets.all(20),
              child: Row(
                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                children: [
                  const Text("SETTINGS", style: TextStyle(color: Colors.white, fontSize: 20, fontWeight: FontWeight.bold, letterSpacing: 2)),
                  IconButton(icon: const Icon(Icons.close, color: Colors.white54), onPressed: onClose)
                ],
              ),
            ),
            const Divider(color: Colors.white10),
            
            // Tabs
            const TabBar(
              isScrollable: true,
              indicatorColor: Color(0xFF2DD4BF),
              labelColor: Color(0xFF2DD4BF),
              unselectedLabelColor: Colors.white54,
              tabs: [
                Tab(text: "KEYS"),
                Tab(text: "CONNECT"),
                Tab(text: "GENERAL"),
                Tab(text: "CONTROLS"),
                Tab(text: "CAMERA"),
                Tab(text: "SAFETY"),
              ],
            ),
            
            // Content
            Expanded(
              child: TabBarView(
                children: [
                  // 1. AI KEYS
                  ListView(padding: const EdgeInsets.all(20), children: [
                     const Text("AI API KEYS (PERSISTENT)", style: TextStyle(color: Color(0xFF2DD4BF), fontSize: 12, fontWeight: FontWeight.bold)),
                     const SizedBox(height: 10),
                     const Text("Keys are saved locally.", style: TextStyle(color: Colors.white38, fontSize: 10)),
                     const SizedBox(height: 20),
                     _ApiKeyInput(provider: "gemini", label: "Gemini Pro Key"),
                     const SizedBox(height: 15),
                     _ApiKeyInput(provider: "openai", label: "OpenAI Key"),
                     const SizedBox(height: 15),
                     _ApiKeyInput(provider: "deepseek", label: "DeepSeek Key"),
                  ]),

                  // 2. CONNECT (NEW)
                  ListView(padding: const EdgeInsets.all(20), children: [
                     const Text("CONNECTION MODE", style: TextStyle(color: Color(0xFF2DD4BF), fontSize: 12, fontWeight: FontWeight.bold)),
                     const SizedBox(height: 15),
                     _SettingRadio(label: "TAILSCALE (Best)", selected: config['connection_mode'] == 'tailscale', onTap: () => set('connection_mode', 'tailscale')),
                     _SettingRadio(label: "CLOUD", selected: config['connection_mode'] == 'cloud', onTap: () => set('connection_mode', 'cloud')),
                     _SettingRadio(label: "LOCAL (Direct WiFi)", selected: config['connection_mode'] == 'local', onTap: () => set('connection_mode', 'local')),
                     _SettingRadio(label: "BLUETOOTH (Coming Soon)", selected: config['connection_mode'] == 'bluetooth', onTap: () => set('connection_mode', 'bluetooth')),

                     if (config['connection_mode'] == 'tailscale') ...[
                        const SizedBox(height: 20),
                        const Text("DRONE TAILSCALE IP", style: TextStyle(color: Colors.white54, fontSize: 10)),
                        const SizedBox(height: 5),
                        _GenericInput(
                          initialValue: config['tailscale_ip'] ?? "100.64.0.30",
                          onChanged: (v) => context.read<CentralState>().updateConfig('tailscale_ip', v),
                          hint: "e.g. 100.64.0.30",
                        ),
                     ],

                     if (config['connection_mode'] == 'local') ...[
                        const SizedBox(height: 20),
                        const Text("LOCAL DRONE IP", style: TextStyle(color: Colors.white54, fontSize: 10)),
                        const SizedBox(height: 5),
                        _GenericInput(
                          initialValue: config['local_ip'] ?? "192.168.4.1",
                          onChanged: (v) => context.read<CentralState>().updateConfig('local_ip', v),
                          hint: "e.g. 192.168.4.1",
                        ),
                     ],

                     const SizedBox(height: 30),
                     Center(child: Text(
                        config['connection_mode'] == 'tailscale'
                          ? "Direct P2P via Tailscale VPN. Works across any network."
                          : config['connection_mode'] == 'local'
                            ? "Ensure phone is connected to Drone Hotspot."
                            : "Uses Internet to route via Cloud Server.",
                        style: const TextStyle(color: Colors.white24, fontSize: 10, fontStyle: FontStyle.italic),
                        textAlign: TextAlign.center,
                     )),
                  ]),

                  // 2. GENERAL
                  ListView(padding: const EdgeInsets.all(20), children: [
                     const Text("UNITS", style: TextStyle(color: Colors.white54, fontSize: 12)),
                     _SettingRadio(label: "Metric (m, km/h)", selected: config['units'] == 'metric', onTap: () => set('units', 'metric')),
                     _SettingRadio(label: "Imperial (ft, mph)", selected: config['units'] == 'imperial', onTap: () => set('units', 'imperial')),
                     const SizedBox(height: 20),
                     // MOVED LOGOUT HERE
                     SizedBox(
                       width: double.infinity,
                       child: OutlinedButton.icon(
                         onPressed: () {
                             Navigator.pushReplacementNamed(context, '/login');
                             context.read<CentralState>().logout();
                         },
                         icon: const Icon(Icons.logout, color: Colors.redAccent, size: 16),
                         label: const Text("LOGOUT SESSION", style: TextStyle(color: Colors.redAccent, fontSize: 12)),
                         style: OutlinedButton.styleFrom(side: const BorderSide(color: Colors.redAccent)),
                       ),
                     ),
                  ]),

                  // 3. CONTROLS
                  ListView(padding: const EdgeInsets.all(20), children: [
                    const Text("JOYSTICK MODE", style: TextStyle(color: Colors.white54, fontSize: 12)),
                    _SettingRadio(label: "Mode 2 (Default)", selected: config['joystick_mode'] == 2, onTap: () => set('joystick_mode', 2)),
                    _SettingRadio(label: "Mode 1", selected: config['joystick_mode'] == 1, onTap: () => set('joystick_mode', 1)),
                    const SizedBox(height: 20),
                    _SettingSlider(label: "Sensitivity", value: (config['sensitivity'] as num).toDouble(), 
                        onChanged: (v) => set('sensitivity', v, sync: false),
                        onChangeEnd: (v) => set('sensitivity', v, sync: true)
                    ),
                    _SettingSlider(label: "Exponential", value: (config['expo'] as num).toDouble(), 
                        onChanged: (v) => set('expo', v, sync: false),
                        onChangeEnd: (v) => set('expo', v, sync: true)
                    ),
                    const SizedBox(height: 10),
                    Center(child: TextButton(onPressed: (){}, child: const Text("CALIBRATE JOYSTICKS", style: TextStyle(color: Color(0xFF2DD4BF))))),
                  ]),

                  // 4. CAMERA
                  ListView(padding: const EdgeInsets.all(20), children: [
                    const Text("SOURCE", style: TextStyle(color: Colors.white54, fontSize: 12)),
                    _SettingRadio(label: "GOPRO HERO 12 (USB-C)", selected: config['cam_source'] == "gopro_usb", onTap: () => set('cam_source', 'gopro_usb')),
                    _SettingRadio(label: "GOPRO HERO 12 (WiFi)", selected: config['cam_source'] == "gopro_wifi", onTap: () => set('cam_source', 'gopro_wifi')),
                    _SettingRadio(label: "ONBOARD (Pi Camera V2.1)", selected: config['cam_source'] == "internal", onTap: () => set('cam_source', 'internal')),
                    _SettingRadio(label: "AUTO (Best Available)", selected: config['cam_source'] == "auto", onTap: () => set('cam_source', 'auto')),
                    const SizedBox(height: 20),
                    const Text("RECORDING QUALITY", style: TextStyle(color: Colors.white54, fontSize: 12)),
                    _SettingRadio(label: "5.3K 30fps (GoPro Hero 12)", selected: config['cap_res'] == "5.3k", onTap: () => set('cap_res', "5.3k")),
                    _SettingRadio(label: "4K 120fps (GoPro Hero 12)", selected: config['cap_res'] == "4k", onTap: () => set('cap_res', "4k")),
                    _SettingRadio(label: "2.7K 240fps (GoPro Hero 12)", selected: config['cap_res'] == "2.7k", onTap: () => set('cap_res', "2.7k")),
                    _SettingRadio(label: "1080p 240fps Slo-Mo (GoPro)", selected: config['cap_res'] == "1080p_gopro", onTap: () => set('cap_res', "1080p_gopro")),
                    const Divider(color: Colors.white10),
                    _SettingRadio(label: "8MP 15fps (Pi V2.1)", selected: config['cap_res'] == "8mp", onTap: () => set('cap_res', "8mp")),
                    _SettingRadio(label: "1080p 30fps (Pi V2.1)", selected: config['cap_res'] == "1080p", onTap: () => set('cap_res', "1080p")),
                    _SettingRadio(label: "720p 60fps (Pi V2.1)", selected: config['cap_res'] == "720p", onTap: () => set('cap_res', "720p")),
                    const SizedBox(height: 20),
                    const Text("GOPRO CONTROLS", style: TextStyle(color: Color(0xFF2DD4BF), fontSize: 12, fontWeight: FontWeight.bold)),
                    const SizedBox(height: 10),
                    Row(children: [
                      Expanded(child: _ActionButton(label: "RECORD", icon: Icons.fiber_manual_record, color: Colors.red, onTap: () => ApiService.sendCommand("GOPRO_SETTINGS", {"action": "record_start"}))),
                      const SizedBox(width: 8),
                      Expanded(child: _ActionButton(label: "STOP", icon: Icons.stop, color: Colors.white54, onTap: () => ApiService.sendCommand("GOPRO_SETTINGS", {"action": "record_stop"}))),
                    ]),
                    const SizedBox(height: 8),
                    Row(children: [
                      Expanded(child: _ActionButton(label: "PHOTO", icon: Icons.camera_alt, color: Colors.white, onTap: () => ApiService.sendCommand("GOPRO_SETTINGS", {"action": "photo"}))),
                      const SizedBox(width: 8),
                      Expanded(child: _ActionButton(label: "TIMELAPSE", icon: Icons.timelapse, color: Colors.amber, onTap: () => ApiService.sendCommand("GOPRO_SETTINGS", {"action": "timelapse"}))),
                    ]),
                    const SizedBox(height: 15),
                    const Text("GOPRO COLOR PROFILE", style: TextStyle(color: Colors.white54, fontSize: 10)),
                    _SettingRadio(label: "Natural", selected: config['gopro_color'] == "natural", onTap: () { set('gopro_color', 'natural'); ApiService.sendCommand("GOPRO_SETTINGS", {"action": "set_color", "value": "natural"}); }),
                    _SettingRadio(label: "Flat (Best for Color Grading)", selected: config['gopro_color'] == "flat", onTap: () { set('gopro_color', 'flat'); ApiService.sendCommand("GOPRO_SETTINGS", {"action": "set_color", "value": "flat"}); }),
                    _SettingRadio(label: "Vibrant", selected: config['gopro_color'] == "vibrant", onTap: () { set('gopro_color', 'vibrant'); ApiService.sendCommand("GOPRO_SETTINGS", {"action": "set_color", "value": "vibrant"}); }),
                  ]),

                  // 5. SAFETY
                  ListView(padding: const EdgeInsets.all(20), children: [
                    _SettingSlider(label: "RTH Altitude (${config['rth_alt']}m)", value: (config['rth_alt'] as num).toDouble() / 100.0, 
                        onChanged: (v) => set('rth_alt', (v * 100).toInt(), sync: false),
                        onChangeEnd: (v) => set('rth_alt', (v * 100).toInt(), sync: true)
                    ),
                    const SizedBox(height: 20),
                    const Text("FAILSAFE BEHAVIORS", style: TextStyle(color: Colors.white54, fontSize: 12)),
                    const SizedBox(height: 10),
                    _SettingSwitch(label: "Obstacle Avoidance", value: config['obstacle_avoidance'], onChanged: (v) => set('obstacle_avoidance', v)),
                    _SettingSwitch(label: "Vision Positioning", value: config['vision_pos'], onChanged: (v) => set('vision_pos', v)),
                    const SizedBox(height: 15),
                    const Text("RETURN TO USER (Not Home)", style: TextStyle(color: Color(0xFF2DD4BF), fontSize: 10)),
                    _SettingRadio(label: "Return to User Location", selected: config['rth_behavior'] == "user", onTap: () => set('rth_behavior', "user")),
                    _SettingRadio(label: "Return to Takeoff Point", selected: config['rth_behavior'] == "home", onTap: () => set('rth_behavior', "home")),
                    const SizedBox(height: 15),
                    // LOW BATTERY
                    // V110: Battery threshold range 5-50% in multiples of 5
                    Text("LOW BATTERY RETURN (${config['batt_threshold'] ?? 20}%)", style: const TextStyle(color: Colors.white54, fontSize: 10)),
                    Slider(
                      value: (config['batt_threshold'] ?? 20).toDouble().clamp(5.0, 50.0),
                      min: 5, max: 50, divisions: 9, // 5,10,15,20,25,30,35,40,45,50
                      activeColor: Colors.redAccent,
                      label: "${config['batt_threshold'] ?? 20}%",
                      onChanged: (v) => set('batt_threshold', v.toInt(), sync: false),
                      onChangeEnd: (v) => set('batt_threshold', v.toInt(), sync: true)
                    ),
                    const SizedBox(height: 10),
                    _SettingRadio(label: "Land at User Location", selected: config['land_behavior'] == "user", onTap: () => set('land_behavior', "user")),
                    _SettingRadio(label: "Land Immediately", selected: config['land_behavior'] == "here", onTap: () => set('land_behavior', "here")),
                  ])
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }
}

class _SettingSwitch extends StatelessWidget {
  final String label;
  final bool value;
  final Function(bool)? onChanged;
  const _SettingSwitch({required this.label, required this.value, this.onChanged});
  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.only(bottom: 15),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceBetween,
        children: [
          Text(label, style: const TextStyle(color: Colors.white)),
          Switch(value: value, onChanged: onChanged, activeColor: const Color(0xFF2DD4BF))
        ],
      ),
    );
  }
}

class _SettingRadio extends StatelessWidget {
  final String label;
  final bool selected;
  final VoidCallback? onTap;
  const _SettingRadio({required this.label, required this.selected, this.onTap});
  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.only(bottom: 10),
      child: GestureDetector(
        onTap: onTap,
        child: Row(
        children: [
          Icon(selected ? Icons.radio_button_checked : Icons.radio_button_unchecked, color: selected ? const Color(0xFF2DD4BF) : Colors.white54, size: 20),
          const SizedBox(width: 10),
          Text(label, style: TextStyle(color: selected ? Colors.white : Colors.white54)),
        ],
        ),
      ),
    );
  }
}

class _SettingSlider extends StatelessWidget {
  final String label;
  final double value;
  final Function(double)? onChanged;
  final Function(double)? onChangeEnd;
  const _SettingSlider({required this.label, required this.value, this.onChanged, this.onChangeEnd});
  @override
  Widget build(BuildContext context) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Text(label, style: const TextStyle(color: Colors.white)),
        Slider(value: value, onChanged: onChanged, onChangeEnd: onChangeEnd, activeColor: const Color(0xFF2DD4BF), inactiveColor: Colors.white10),
      ],
    );
  }
}

class _ApiKeyInput extends StatefulWidget {
  final String provider;
  final String label;
  const _ApiKeyInput({required this.provider, required this.label});

  @override
  State<_ApiKeyInput> createState() => _ApiKeyInputState();
}

class _ApiKeyInputState extends State<_ApiKeyInput> {
  final _controller = TextEditingController();

  @override
  void initState() {
    super.initState();
    _loadKey();
  }

  Future<void> _loadKey() async {
    final prefs = await SharedPreferences.getInstance();
    final key = prefs.getString('api_key_${widget.provider}') ?? "";
    if (mounted) setState(() => _controller.text = key);
  }

  Future<void> _saveKey(String value) async {
    final prefs = await SharedPreferences.getInstance();
    await prefs.setString('api_key_${widget.provider}', value);
    
    // ⚡ SYNC TO LIVE STATE IMMEDIATELY
    if (mounted) {
       final keyName = "${widget.provider}_api_key"; // gemini_api_key or openai_api_key
       context.read<CentralState>().updateConfig(keyName, value);
    }
  }

  @override
  Widget build(BuildContext context) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Text(widget.label, style: const TextStyle(color: Colors.white, fontSize: 12)),
        const SizedBox(height: 5),
        TextField(
          controller: _controller,
          style: const TextStyle(color: Colors.white, fontSize: 12),
          decoration: InputDecoration(
            filled: true,
            fillColor: Colors.white10,
            isDense: true,
            border: OutlineInputBorder(borderRadius: BorderRadius.circular(8), borderSide: BorderSide.none),
            hintText: "Enter Key...",
            hintStyle: TextStyle(color: Colors.white.withOpacity(0.3)),
          ),
          onChanged: _saveKey,
        )
      ],
    );
  }
}

class _GenericInput extends StatefulWidget {
  final String initialValue;
  final ValueChanged<String> onChanged;
  final String hint;
  
  const _GenericInput({required this.initialValue, required this.onChanged, this.hint = ""});
  
  @override
  State<_GenericInput> createState() => _GenericInputState();
}

class _GenericInputState extends State<_GenericInput> {
  late TextEditingController _controller;
  
  @override
  void initState() {
    super.initState();
    _controller = TextEditingController(text: widget.initialValue);
  }
  
  @override
  Widget build(BuildContext context) {
    return TextField(
      controller: _controller,
      style: const TextStyle(color: Colors.white, fontSize: 12),
      decoration: InputDecoration(
        filled: true,
        fillColor: Colors.white10,
        isDense: true,
        border: OutlineInputBorder(borderRadius: BorderRadius.circular(8), borderSide: BorderSide.none),
        hintText: widget.hint,
        hintStyle: TextStyle(color: Colors.white.withOpacity(0.3)),
      ),
      onChanged: widget.onChanged,
    );
  }
}

class _ActionButton extends StatelessWidget {
  final String label;
  final IconData icon;
  final Color color;
  final VoidCallback onTap;

  const _ActionButton({required this.label, required this.icon, required this.color, required this.onTap});

  @override
  Widget build(BuildContext context) {
    return Material(
      color: Colors.white.withOpacity(0.05),
      borderRadius: BorderRadius.circular(8),
      child: InkWell(
        onTap: onTap,
        borderRadius: BorderRadius.circular(8),
        child: Padding(
          padding: const EdgeInsets.symmetric(vertical: 12),
          child: Column(
            children: [
              Icon(icon, color: color, size: 20),
              const SizedBox(height: 4),
              Text(label, style: TextStyle(color: color, fontSize: 9, fontWeight: FontWeight.bold)),
            ],
          ),
        ),
      ),
    );
  }
}
