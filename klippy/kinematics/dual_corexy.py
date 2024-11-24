# File: klippy/kinematics/dual_corexy.py

import logging
import stepper
from . import idex_modes

class DualCoreXYKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()
        self.toolhead = toolhead
        
        # Register steppers for top toolhead
        self.rails_top = [
            stepper.LookupMultiRail(config.getsection('stepper_top_left')),
            stepper.LookupMultiRail(config.getsection('stepper_top_right')),
            stepper.LookupMultiRail(config.getsection('stepper_z'))
        ]
        
        # Register steppers for bottom toolhead
        self.rails_bottom = [
            stepper.LookupMultiRail(config.getsection('stepper_bottom_left')),
            stepper.LookupMultiRail(config.getsection('stepper_bottom_right')),
            stepper.LookupMultiRail(config.getsection('stepper_z2'))
        ]
        
        # Read axis configuration
        self.max_velocity = config.getfloat('max_velocity', 300., above=0.)
        self.max_accel = config.getfloat('max_accel', 3000., above=0.)
        self.max_z_velocity = config.getfloat('max_z_velocity', 25., above=0.)
        self.max_z_accel = config.getfloat('max_z_accel', 100., above=0.)
        
        # Wire tension parameters
        self.min_wire_tension = config.getfloat('min_wire_tension', 0.5, above=0.)
        self.max_wire_angle = config.getfloat('max_wire_angle', 5., above=0.)
        
        # Initialize positions
        self.setup_rails()
        self.build_registers()
        
    def setup_rails(self):
        """Initialize rail configurations"""
        for rail in self.rails_top + self.rails_bottom:
            rail.setup_itersolve('corexy_stepper_alloc')
            rail.setup_range((-1000., 1000.))
        # Register STOP_ON_ERROR if needed
        # ...
        
    def build_registers(self):
        """Register commands and handlers"""
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('SET_WIRE_TENSION', self.cmd_SET_WIRE_TENSION,
                             desc=self.cmd_SET_WIRE_TENSION_help)
    
    def get_status(self, eventtime):
        """Return status of dual CoreXY system"""
        status = {
            'top_pos': self.get_top_position(),
            'bottom_pos': self.get_bottom_position(),
            'wire_angle': self.calculate_wire_angle(),
            'wire_tension': self.estimate_wire_tension()
        }
        return status
        
    def _forward_kinematics(self, left, right):
        """Convert stepper positions to cartesian coordinates for CoreXY"""
        x = (left + right) * 0.5
        y = (right - left) * 0.5
        return [x, y]
        
    def _inverse_kinematics(self, x, y):
        """Convert cartesian coordinates to stepper positions for CoreXY"""
        left = x - y
        right = x + y
        return [left, right]
        
    def get_top_position(self):
        """Get cartesian position of top toolhead"""
        left = self.rails_top[0].get_commanded_position()
        right = self.rails_top[1].get_commanded_position()
        z = self.rails_top[2].get_commanded_position()
        xy = self._forward_kinematics(left, right)
        return [xy[0], xy[1], z]
        
    def get_bottom_position(self):
        """Get cartesian position of bottom toolhead"""
        left = self.rails_bottom[0].get_commanded_position()
        right = self.rails_bottom[1].get_commanded_position()
        z = self.rails_bottom[2].get_commanded_position()
        xy = self._forward_kinematics(left, right)
        return [xy[0], xy[1], z]
        
    def calculate_wire_angle(self):
        """Calculate current wire angle from vertical"""
        top_pos = self.get_top_position()
        bottom_pos = self.get_bottom_position()
        dx = top_pos[0] - bottom_pos[0]
        dy = top_pos[1] - bottom_pos[1]
        dz = top_pos[2] - bottom_pos[2]
        import math
        return math.degrees(math.atan2(math.sqrt(dx*dx + dy*dy), dz))
        
    def estimate_wire_tension(self):
        """Estimate current wire tension based on positions"""
        # Implement wire tension calculation based on your specific setup
        # This is a placeholder
        return self.min_wire_tension
        
    def check_move(self, move):
        """Check if move is valid for both toolheads"""
        # Verify wire angle limits
        if self.calculate_wire_angle() > self.max_wire_angle:
            raise self.printer.command_error(
                "Move exceeds maximum wire angle of %.1f degrees" % (
                    self.max_wire_angle))
        
        # Check individual axis limits
        # ... (implement boundary checking)
        
    def move(self, move):
        """Generate step times for a coordinated move"""
        # Calculate top toolhead move
        top_end = move.end_pos[:3]  # First three coordinates for top
        top_stepper_pos = self._inverse_kinematics(top_end[0], top_end[1])
        
        # Calculate bottom toolhead move
        bottom_end = move.end_pos[3:6]  # Next three coordinates for bottom
        bottom_stepper_pos = self._inverse_kinematics(bottom_end[0], bottom_end[1])
        
        # Generate step times for all steppers
        for i, rail in enumerate(self.rails_top):
            rail.step_itersolve(move.cmove)
        for i, rail in enumerate(self.rails_bottom):
            rail.step_itersolve(move.cmove)
            
    cmd_SET_WIRE_TENSION_help = "Adjust wire tension parameters"
    def cmd_SET_WIRE_TENSION(self, gcmd):
        """Set wire tension parameters"""
        tension = gcmd.get_float('TENSION', self.min_wire_tension, minval=0.)
        self.min_wire_tension = tension
        gcmd.respond_info("Wire tension set to %.3f" % (tension,))

def load_kinematics(toolhead, config):
    """Load dual CoreXY kinematics"""
    return DualCoreXYKinematics(toolhead, config)