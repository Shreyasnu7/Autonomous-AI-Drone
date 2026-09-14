"""
Drone Hardware Configuration System
Provides AI modules with real-time hardware specifications for intelligent flight planning.

Data Sources:
- Static specs (weight, frame, motors) - hardcoded
- Dynamic params (hover throttle, battery) - from FC via Radxa telemetry

Usage in AI modules:
    from drone_config import DroneConfig
    
    # Access specs
    weight = DroneConfig.DRONE_WEIGHT  # kg
    hover_pwm = DroneConfig.get_hover_pwm()  # 1000-2000
    
    # Update from telemetry (called automatically by director_core)
    DroneConfig.update_from_fc_telemetry(fc_params)
"""

import threading
import time

class DroneConfig:
    """
    Centralized drone hardware configuration accessible by all AI modules.
    Thread-safe singleton pattern for real-time telemetry updates.
    """
    
    # === STATIC HARDWARE SPECS ===
    # Updated by User: DJI F450 Setup + GoPro Hero 13
    
    ## Physical Properties
    # Drone (1.3kg) + GoPro Hero 13 (~300g) = ~1.6kg (APPROXIMATE - NOT USED FOR PHYSICS CALCS)
    DRONE_WEIGHT = 1.6  # kg
    FRAME_TYPE = "F450"
    MOTOR_COUNT = 4
    MOTOR_MODEL = "DJI 2212 920KV"
    MOTOR_KV = 920
    ESC_RATING = 30 # Amps
    PROP_SIZE = 10  # inches (10x4.5 props)
    MOTOR_SPACING = 450  # mm (wheelbase)
    
    ## Battery Configuration
    BATTERY_CAPACITY = 5400  # mAh
    BATTERY_CELLS = 3  # 3S Li-Po
    BATTERY_C_RATING = 60 # C
    BATTERY_TYPE = "Li-Po" 
    BATTERY_MIN_VOLTAGE = 10.5  # V (3.5V per cell - Safe for LiPo under load)
    BATTERY_MAX_VOLTAGE = 12.6  # V (4.2V per cell)
    BATTERY_NOMINAL_VOLTAGE = 11.1  # V (3.7V per cell)
    
    ## Flight Characteristics (from FC params in bridge lines 134-158)
    MAX_TILT_ANGLE = 60  # degrees (ANGLE_MAX=6000 centidegrees)
    MAX_CLIMB_RATE = 5.0  # m/s (PILOT_SPEED_UP=500 cm/s)
    MAX_DESCENT_RATE = 1.5  # m/s (standard safe descent)
    MAX_YAW_RATE = 60  # deg/s (typical for F450)
    
    # === DYNAMIC FC PARAMETERS ===
    # (Updated from FC telemetry via Radxa)
    
    _lock = threading.Lock()
    
    # Hover throttle (learned by FC in flight)
    _hover_throttle = 0.55  # Default for 1.5kg F450
    _hover_throttle_timestamp = 0
    
    # Battery state (real-time)
    _battery_voltage = 11.1  # V
    _battery_percent = 100  # %
    _battery_current = 0.0  # A
    _battery_timestamp = 0
    
    # Flight mode
    _flight_mode = "STABILIZE"
    _is_armed = False
    
    # GPS/Position
    _gps_fix = False
    _satellites = 0
    _altitude_agl = 0.0  # m (above ground level)
    _groundspeed = 0.0  # m/s
    
    @classmethod
    def update_from_fc_telemetry(cls, fc_telem: dict):
        """
        Update dynamic params from FC telemetry (relayed by Radxa).
        
        Args:
            fc_telem: Dictionary from FC containing:
                - MOT_THST_HOVER: Learned hover throttle (0.0-1.0)
                - voltage: Battery voltage (V)
                - battery: Battery percent (0-100)
                - current: Battery current draw (A)
                - mode_id: Flight mode string
                - armed: Armed state (bool)
                - sats: GPS satellite count
                - altitude: Altitude AGL (m)
                - speed: Ground speed (m/s)
        """
        with cls._lock:
            current_time = time.time()
            
            # Hover throttle (critical for AI speed calculations)
            if 'MOT_THST_HOVER' in fc_telem:
                new_hover = float(fc_telem['MOT_THST_HOVER'])
                # Sanity check: typical range 0.3-0.8
                if 0.1 < new_hover < 0.9:
                    cls._hover_throttle = new_hover
                    cls._hover_throttle_timestamp = current_time
            
            # Battery state
            if 'voltage' in fc_telem:
                cls._battery_voltage = float(fc_telem['voltage'])
                cls._battery_timestamp = current_time
            
            if 'battery' in fc_telem:
                cls._battery_percent = int(fc_telem['battery'])
            
            if 'current' in fc_telem:
                cls._battery_current = float(fc_telem.get('current', 0))
            
            # Flight state
            if 'mode_id' in fc_telem:
                cls._flight_mode = str(fc_telem['mode_id'])
            
            if 'armed' in fc_telem:
                cls._is_armed = bool(fc_telem['armed'])
            
            # GPS/Position
            if 'sats' in fc_telem:
                cls._satellites = int(fc_telem['sats'])
                cls._gps_fix = cls._satellites >= 6
            
            if 'altitude' in fc_telem:
                cls._altitude_agl = float(fc_telem['altitude'])
            
            if 'speed' in fc_telem:
                cls._groundspeed = float(fc_telem['speed'])
    
    # === GETTER METHODS FOR AI MODULES ===
    
    @classmethod
    def get_hover_throttle(cls) -> float:
        """Get learned hover throttle (0.0-1.0)"""
        with cls._lock:
            return cls._hover_throttle
    
    @classmethod
    def get_hover_pwm(cls) -> int:
        """Get hover throttle as PWM value (1000-2000)"""
        return int(1000 + (cls.get_hover_throttle() * 1000))
    
    @classmethod
    def get_battery_status(cls) -> dict:
        """Get current battery status"""
        with cls._lock:
            return {
                'voltage': cls._battery_voltage,
                'percent': cls._battery_percent,
                'current': cls._battery_current,
                'remaining_mah': (cls._battery_percent / 100.0) * cls.BATTERY_CAPACITY,
                'is_critical': cls._battery_percent < 15,
                'is_low': cls._battery_percent < 30,
                'timestamp': cls._battery_timestamp
            }
    
    @classmethod
    def get_flight_state(cls) -> dict:
        """Get current flight state"""
        with cls._lock:
            return {
                'mode': cls._flight_mode,
                'is_armed': cls._is_armed,
                'gps_fix': cls._gps_fix,
                'satellites': cls._satellites,
                'altitude_agl': cls._altitude_agl,
                'groundspeed': cls._groundspeed
            }
    
    @classmethod
    def is_safe_to_fly(cls) -> tuple[bool, str]:
        """
        Check if drone is safe to fly based on hardware status.
        Returns: (is_safe: bool, reason: str)
        """
        battery = cls.get_battery_status()
        
        if battery['is_critical']:
            return False, f"Battery critical: {battery['percent']}%"
        
        if cls._battery_voltage < cls.BATTERY_MIN_VOLTAGE:
            return False, f"Battery voltage too low: {cls._battery_voltage:.1f}V"
        
        # All checks passed
        return True, "OK"
    
    @classmethod
    def calculate_max_speed(cls, current_altitude: float = None) -> float:
        """
        Calculate safe maximum horizontal speed based on REAL-TIME FC TELEMETRY.
        Uses learned Hover Throttle (MOT_THST_HOVER) to estimate Thrust-to-Weight Ratio (TWR).
        
        Args:
            current_altitude: Current AGL altitude in meters
            
        Returns:
            Maximum safe horizontal speed in m/s
        """
        # 1. Estimate Thrust-to-Weight Ratio (TWR) from Hover Throttle
        # Hover Throttle 0.5 means TWR = 1/0.5 = 2.0 (2x thrust available)
        # Hover Throttle 0.8 means TWR = 1/0.8 = 1.25 (Heavy! Low performance)
        hover_throttle = cls.get_hover_throttle()
        
        # Safety clamp: Avoid divide by zero or unrealistic values
        if hover_throttle < 0.1: hover_throttle = 0.5 # Default if FC hasn't learned yet
        
        twr = 1.0 / hover_throttle
        
        # 2. Maximum safe tilt angle calculation
        # To maintain altitude, Thrust_Vertical = Weight
        # Thrust_Total * cos(max_tilt) = Weight
        # cos(max_tilt) = Weight / Thrust_Total = 1 / TWR
        # max_tilt = arccos(1/TWR)
        
        # We limit max tilt to 45 degrees for safety, or less if TWR is low
        import math
        try:
            theoretical_max_tilt_rad = math.acos(1.0 / twr)
            theoretical_max_tilt_deg = math.degrees(theoretical_max_tilt_rad)
        except ValueError:
            theoretical_max_tilt_deg = 0 # Updates when flying
            
        # Cap at user-defined safety limit (User wanted 60, but let's be safe at 45 for autonomous)
        safe_tilt_limit = min(theoretical_max_tilt_deg, 45.0, cls.MAX_TILT_ANGLE)
        
        # 3. Convert Tilt to Max Speed (Rough Aerodynamic Estimate)
        # V_max ~= sqrt(2 * Horizontal_Thrust / (Cd * Area * Rho))
        # Simplified linear map for F450: 45deg tilt ~= 15 m/s
        
        base_max_speed = (safe_tilt_limit / 45.0) * 15.0
        
        # 4. Apply Derating Factors
        
        # Battery Voltage Sag
        battery = cls.get_battery_status()
        if battery['is_low']:
            base_max_speed *= 0.6  # 40% reduction for low battery
        
        # Altitude (Air Density)
        if current_altitude and current_altitude > 100:
            base_max_speed *= 0.85
            
        # FC Constraints (Don't exceed what FC considers safe)
        # We don't have this direct params unless read from params list, 
        # but we respect the hard limit set in this config
        
        return max(2.0, base_max_speed) # Minimum 2.0 m/s so it doesn't get stuck
    
    @classmethod
    def calculate_flight_time_remaining(cls) -> float:
        """
        Estimate remaining flight time in minutes based on current draw.
        
        Returns:
            Estimated minutes until battery critical (15%)
        """
        battery = cls.get_battery_status()
        
        if battery['current'] <= 0:
            # No current draw data, use conservative estimate
            # At hover: ~15A for F450 1.5kg
            # 5400mAh 3S 60C / 15A = 0.36h = ~21min full battery
            estimated_current = 15.0  # A
        else:
            estimated_current = battery['current']
        
        # Remaining capacity until 15%
        usable_percent = battery['percent'] - 15
        if usable_percent <= 0:
            return 0.0
        
        usable_mah = (usable_percent / 100.0) * cls.BATTERY_CAPACITY
        usable_ah = usable_mah / 1000.0
        
        # Time = Capacity / Current
        hours = usable_ah / estimated_current
        minutes = hours * 60
        
        return minutes
    
    @classmethod
    def get_all_specs(cls) -> dict:
        """Get complete drone specification snapshot for logging/debugging"""
        return {
            'hardware': {
                'weight_kg': cls.DRONE_WEIGHT,
                'frame': cls.FRAME_TYPE,
                'motors': cls.MOTOR_COUNT,
                'motor_kv': cls.MOTOR_KV,
                'prop_size_inches': cls.PROP_SIZE,
                'battery_capacity_mah': cls.BATTERY_CAPACITY,
                'battery_type': cls.BATTERY_TYPE,
                'battery_cells': cls.BATTERY_CELLS
            },
            'limits': {
                'max_tilt_deg': cls.MAX_TILT_ANGLE,
                'max_climb_ms': cls.MAX_CLIMB_RATE,
                'max_descent_ms': cls.MAX_DESCENT_RATE,
                'max_yaw_rate_degs': cls.MAX_YAW_RATE
            },
            'dynamic': {
                'hover_throttle': cls.get_hover_throttle(),
                'hover_pwm': cls.get_hover_pwm(),
                **cls.get_battery_status(),
                **cls.get_flight_state()
            }
        }
