import stepper
from . import idex_modes


class DualCartKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()
        # Setup primary rails
        self.rails = []
        # Setup X and Y primary steppers
        for axis in "xy":
            self.rails.append(
                stepper.LookupMultiRail(config.getsection(f"stepper_{axis}"))
            )
        
        # Setup secondary steppers if configured
        self.secondary_rails = []
        for axis in "xy":
            section_name = f"stepper_{axis}2"
            if config.has_section(section_name):
                rail = stepper.LookupMultiRail(config.getsection(section_name))
                self.secondary_rails.append(rail)
            else:
                self.secondary_rails.append(None)
        
        # Configure all rails
        for rail, axis in zip(self.rails, "xy"):
            rail.setup_itersolve("cartesian_stepper_alloc", axis.encode())
        for rail, axis in zip(self.secondary_rails, "xy"):
            if rail is not None:
                rail.setup_itersolve("cartesian_stepper_alloc", axis.encode())
        
        # Set up position ranges
        ranges = [r.get_range() for r in self.rails]
        self.axes_min = toolhead.Coord(*[r[0] for r in ranges], z=0.0, e=0.0)
        self.axes_max = toolhead.Coord(*[r[1] for r in ranges], z=0.0, e=0.0)
        
        # Set up steppers
        all_rails = [r for r in self.rails + self.secondary_rails if r is not None]
        for s in [s for rail in all_rails for s in rail.get_steppers()]:
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        
        # Register motor off handler
        self.printer.register_event_handler(
            "stepper_enable:motor_off", self._motor_off
        )
        
        # Setup boundary checks
        self.limits = [(1.0, -1.0)] * 2  # Only X and Y limits
        
        # Store rail configurations
        self.has_secondary = {
            'x': self.secondary_rails[0] is not None,
            'y': self.secondary_rails[1] is not None
        }

    def get_steppers(self):
        steppers = []
        for rail in self.rails + self.secondary_rails:
            if rail is not None:
                steppers.extend(rail.get_steppers())
        return steppers

    def calc_position(self, stepper_positions):
        positions = []
        for i, rail in enumerate(self.rails):
            positions.append(stepper_positions[rail.get_name()])
            if self.secondary_rails[i] is not None:
                # For now, we use the primary rail's position
                # You might want to modify this based on your needs
                pass
        return positions + [0.0]  # Add Z=0

    def update_limits(self, i, range):
        if i >= len(self.limits):
            return
        l, h = self.limits[i]
        if l <= h:
            self.limits[i] = range

    def set_position(self, newpos, homing_axes):
        for rail in self.rails:
            rail.set_position(newpos)
        for rail in self.secondary_rails:
            if rail is not None:
                rail.set_position(newpos)
        for axis in homing_axes:
            if axis >= len(self.rails):
                continue
            self.limits[axis] = self.rails[axis].get_range()
            if self.secondary_rails[axis] is not None:
                secondary_range = self.secondary_rails[axis].get_range()
                # You might want to handle this differently based on your needs
                self.limits[axis] = (
                    min(self.limits[axis][0], secondary_range[0]),
                    max(self.limits[axis][1], secondary_range[1])
                )

    def home_axis(self, homing_state, axis, rails_to_home):
        # Home multiple rails for the same axis if needed
        position_min = min(r.get_range()[0] for r in rails_to_home)
        position_max = max(r.get_range()[1] for r in rails_to_home)
        
        # Use the first rail's homing info
        hi = rails_to_home[0].get_homing_info()
        homepos = [None, None, None, None]
        homepos[axis] = hi.position_endstop
        forcepos = list(homepos)
        
        if hi.positive_dir:
            forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
        else:
            forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
        
        homing_state.home_rails(rails_to_home, forcepos, homepos)

    def home(self, homing_state):
        # Home each axis independently
        for axis in homing_state.get_axes():
            if axis >= len(self.rails):
                continue
            rails_to_home = [self.rails[axis]]
            if self.secondary_rails[axis] is not None:
                rails_to_home.append(self.secondary_rails[axis])
            self.home_axis(homing_state, axis, rails_to_home)

    def _motor_off(self, print_time):
        self.limits = [(1.0, -1.0)] * 2

    def _check_endstops(self, move):
        end_pos = move.end_pos
        for i in (0, 1):  # Only check X and Y
            if move.axes_d[i] and (
                end_pos[i] < self.limits[i][0] or end_pos[i] > self.limits[i][1]
            ):
                if self.limits[i][0] > self.limits[i][1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()

    def check_move(self, move):
        limits = self.limits
        xpos, ypos = move.end_pos[:2]
        if (
            xpos < limits[0][0]
            or xpos > limits[0][1]
            or ypos < limits[1][0]
            or ypos > limits[1][1]
        ):
            self._check_endstops(move)

    def get_status(self, eventtime):
        axes = [a for a, (l, h) in zip("xy", self.limits) if l <= h]
        return {
            "homed_axes": "".join(axes),
            "axis_minimum": self.axes_min,
            "axis_maximum": self.axes_max,
            "has_x2": self.has_secondary['x'],
            "has_y2": self.has_secondary['y']
        }


def load_kinematics(toolhead, config):
    return DualCartKinematics(toolhead, config)
