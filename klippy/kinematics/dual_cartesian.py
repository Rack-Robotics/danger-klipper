# Code for handling the kinematics of dual independent cartesian systems for Wire EDM
#
# Based on Klipper cartesian.py by Kevin O'Connor <kevin@koconnor.net>
# Modified for Wire EDM dual independent cartesian systems
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import stepper

class EDMCoord:
    def __init__(self, x, y, u, v, e=0.):
        self.x, self.y = x, y
        self.u, self.v = u, v
        self.e = e
    def __getitem__(self, i):
        if i == 0: return self.x
        if i == 1: return self.y
        if i == 2: return self.u
        if i == 3: return self.v
        if i == 4: return self.e
        raise IndexError
    def __add__(self, other):
        return EDMCoord(self.x + other[0], self.y + other[1],
                       self.u + other[2], self.v + other[3],
                       self.e + other[4])
    def __sub__(self, other):
        return EDMCoord(self.x - other[0], self.y - other[1],
                       self.u - other[2], self.v - other[3],
                       self.e - other[4])
    def __mul__(self, other):
        return EDMCoord(self.x * other, self.y * other,
                       self.u * other, self.v * other,
                       self.e * other)
    def __truediv__(self, other):
        return EDMCoord(self.x / other, self.y / other,
                       self.u / other, self.v / other,
                       self.e / other)
    def __str__(self):
        return "(%s, %s, %s, %s, %s)" % (self.x, self.y, self.u, self.v, self.e)
    def __repr__(self):
        return self.__str__()
    def as_list(self):
        return [self.x, self.y, self.u, self.v, self.e]

class WireEDMKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()
        # Setup axis rails for both cartesian systems
        # First system uses X,Y axes
        # Second system uses U,V axes (representing second X,Y)
        self.rails = [
            stepper.LookupMultiRail(config.getsection('stepper_' + n))
            for n in 'xyuv'
        ]
        
        # Setup itersolve for each rail
        # X and U are horizontal motion
        # Y and V are vertical motion
        for rail, axis in zip(self.rails[:2], 'xy'):
            rail.setup_itersolve('cartesian_stepper_alloc', axis.encode())
        # U and V rails are set up as additional X/Y axes
        self.rails[2].setup_itersolve('cartesian_stepper_alloc', 'x'.encode())  # U rail
        self.rails[3].setup_itersolve('cartesian_stepper_alloc', 'y'.encode())  # V rail
            
        # Get range of motion for each rail
        ranges = [r.get_range() for r in self.rails]
        
        # Initialize min/max coordinates
        self.axes_min = EDMCoord(ranges[0][0], ranges[1][0],
                                ranges[2][0], ranges[3][0], 0.)
        self.axes_max = EDMCoord(ranges[0][1], ranges[1][1],
                                ranges[2][1], ranges[3][1], 0.)
        
        # Register steppers
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
            
        self.printer.register_event_handler(
            "stepper_enable:motor_off", self._motor_off
        )
        
        # Setup boundary checks
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.limits = [(1.0, -1.0)] * 4  # Limits for X,Y,U,V axes

    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]

    def calc_position(self, stepper_positions):
        return [stepper_positions[rail.get_name()] for rail in self.rails]

    def update_limits(self, i, range):
        l, h = self.limits[i]
        # Only update limits if this axis was already homed
        if l <= h:
            self.limits[i] = range

    def set_position(self, newpos, homing_axes):
        for i, rail in enumerate(self.rails):
            rail.set_position(newpos)
            if i in homing_axes:
                self.limits[i] = rail.get_range()

    def home(self, homing_state):
        # Each axis is homed independently and in order
        for axis in homing_state.get_axes():
            self.home_axis(homing_state, axis, self.rails[axis])

    def home_axis(self, homing_state, axis, rail):
        # Determine movement
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()
        homepos = [None, None, None, None]
        homepos[axis] = hi.position_endstop
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
        else:
            forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
        # Perform homing
        homing_state.home_rails([rail], forcepos, homepos)

    def _motor_off(self, print_time):
        self.limits = [(1.0, -1.0)] * 4

    def _check_endstops(self, move):
        end_pos = move.end_pos
        for i in (0, 1, 2, 3):  # Check X,Y,U,V axes
            if (move.axes_d[i] and
                (end_pos[i] < self.limits[i][0] or end_pos[i] > self.limits[i][1])):
                if self.limits[i][0] > self.limits[i][1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()

    def check_move(self, move):
        limits = self.limits
        # Check both X,Y and U,V positions
        xpos, ypos, upos, vpos = move.end_pos[:4]
        
        # Check primary XY system limits
        if (xpos < limits[0][0] or xpos > limits[0][1] or
            ypos < limits[1][0] or ypos > limits[1][1]):
            self._check_endstops(move)
            
        # Check secondary UV system limits
        if (upos < limits[2][0] or upos > limits[2][1] or
            vpos < limits[3][0] or vpos > limits[3][1]):
            self._check_endstops(move)

    def get_status(self, eventtime=None):
        axes = [a for a, (l, h) in zip("xyuv", self.limits) if l <= h]
        return {
            "homed_axes": "".join(axes),
            "axis_minimum": self.axes_min.as_list(),
            "axis_maximum": self.axes_max.as_list(),
        }

def load_kinematics(toolhead, config):
    return WireEDMKinematics(toolhead, config)