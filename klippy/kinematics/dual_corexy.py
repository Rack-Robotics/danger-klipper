# Code for handling the kinematics of dual CoreXY systems for Wire EDM
#
# Based on Klipper corexy.py by Kevin O'Connor <kevin@koconnor.net>
# Modified for Wire EDM dual independent CoreXY systems
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

class DualCoreXYKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()
        # CoreXY A/B motors control X+Y (1st gantry) or U+V (2nd gantry) movement
        self.rails = [stepper.LookupMultiRail(config.getsection(name + '_stepper'))
                     for name in ('xa', 'xb', 'ua', 'ub')]
        
        # Setup itersolve for CoreXY steppers
        # First CoreXY system (X/Y)
        self.rails[0].setup_itersolve('corexy_stepper_alloc', b'A')
        self.rails[1].setup_itersolve('corexy_stepper_alloc', b'B')
        # Second CoreXY system (U/V)
        self.rails[2].setup_itersolve('corexy_stepper_alloc', b'A')
        self.rails[3].setup_itersolve('corexy_stepper_alloc', b'B')
        
        # Define positions
        ranges = [r.get_range() for r in self.rails]
        self.axes_min = EDMCoord(-ranges[0][1], -ranges[0][1],  # X/Y minimum
                                -ranges[2][1], -ranges[2][1], 0.) # U/V minimum
        self.axes_max = EDMCoord(ranges[1][1], ranges[1][1],    # X/Y maximum
                                ranges[3][1], ranges[3][1], 0.)  # U/V maximum
        
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        
        # Setup boundary checks
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.limits = [(1.0, -1.0)] * 4  # Limits for X,Y,U,V axes
        
        self.printer.register_event_handler(
            "stepper_enable:motor_off", self._motor_off)

    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]

    def calc_position(self, stepper_positions):
        # First CoreXY position
        xa_pos = stepper_positions[self.rails[0].get_name()]
        xb_pos = stepper_positions[self.rails[1].get_name()]
        x_pos = (xa_pos + xb_pos) * .5
        y_pos = (xa_pos - xb_pos) * .5
        # Second CoreXY position
        ua_pos = stepper_positions[self.rails[2].get_name()]
        ub_pos = stepper_positions[self.rails[3].get_name()]
        u_pos = (ua_pos + ub_pos) * .5
        v_pos = (ua_pos - ub_pos) * .5
        return [x_pos, y_pos, u_pos, v_pos]

    def set_position(self, newpos, homing_axes):
        for i, rail in enumerate(self.rails):
            if i < 2:
                # First CoreXY - X+Y
                x_pos, y_pos = newpos[:2]
                if i == 0:
                    rail.set_position(x_pos + y_pos)  # xa = x + y
                else:
                    rail.set_position(x_pos - y_pos)  # xb = x - y
            else:
                # Second CoreXY - U+V
                u_pos, v_pos = newpos[2:4]
                if i == 2:
                    rail.set_position(u_pos + v_pos)  # ua = u + v
                else:
                    rail.set_position(u_pos - v_pos)  # ub = u - v
            
            # Update limits for homed axes
            if i < 2 and 0 in homing_axes:
                self.limits[0] = rail.get_range()
            if i < 2 and 1 in homing_axes:
                self.limits[1] = rail.get_range()
            if i >= 2 and 2 in homing_axes:
                self.limits[2] = rail.get_range()
            if i >= 2 and 3 in homing_axes:
                self.limits[3] = rail.get_range()

    def _home_axis(self, homing_state, axis, rail):
        # Determine if this axis should home positive or negative
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()
        homepos = [None, None, None, None]
        homepos[axis] = hi.position_endstop
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
        else:
            forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
        # Home axis
        homing_state.home_rails([rail], forcepos, homepos)

    def home(self, homing_state):
        # Home each axis
        for axis in homing_state.get_axes():
            if axis == 0:
                # Home X - use both XA and XB steppers
                self._home_axis(homing_state, axis, self.rails[0])
                self._home_axis(homing_state, axis, self.rails[1])
            elif axis == 1:
                # Home Y - use both XA and XB steppers
                self._home_axis(homing_state, axis, self.rails[0])
                self._home_axis(homing_state, axis, self.rails[1])
            elif axis == 2:
                # Home U - use both UA and UB steppers
                self._home_axis(homing_state, axis, self.rails[2])
                self._home_axis(homing_state, axis, self.rails[3])
            elif axis == 3:
                # Home V - use both UA and UB steppers
                self._home_axis(homing_state, axis, self.rails[2])
                self._home_axis(homing_state, axis, self.rails[3])

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
    return DualCoreXYKinematics(toolhead, config)