import multiprocessing as mp
import sys

from ctypes import c_bool, c_double
from PyQt5 import QtWidgets

from ergocycleS2M.gui.gui import ErgocycleGUI
from ergocycleS2M.motor_control.enums import ControlMode
# from ergocycleS2M.motor_control.motor_controller import MotorController
from ergocycleS2M.motor_control.mock_controller import MockController


class Application:
    """ """

    def __init__(self):
        # Shared memory
        # Instructions
        self.instruction = mp.Manager().Value(c_double, 0.0)
        self.ramp_instruction = mp.Manager().Value(c_double, 0.0)
        self.spin_box = mp.Manager().Value(c_double, 0.0)
        # State of the motor
        self.run = mp.Manager().Value(c_bool, True)
        self.stopping = mp.Manager().Value(c_bool, False)
        self.queue_instructions = mp.Manager().Queue()
        # Motor values
        self.zero_position = mp.Manager().Value(c_bool, 0.0)
        self.i_measured = mp.Manager().Value(c_double, 0.0)
        self.turns = mp.Manager().Value(c_double, 0.0)
        self.vel_estimate = mp.Manager().Value(c_double, 0.0)
        # Saving
        self.saving = mp.Manager().Value(c_bool, False)

        # Pipe
        self.queue_comment = mp.Queue()

        # Create the processes
        self.motor_process = mp.Process(name="motor", target=self.motor_control_process, daemon=True)
        self.gui_process = mp.Process(name="gui", target=self.gui_process, daemon=True)

    def start(self):
        """ """
        self.motor_process.start()
        self.gui_process.start()
        self.gui_process.join()

    def motor_control_process(self):
        """
        Main loop of the thread. It is called when the thread is started. It is stopped when the thread is stopped.
        It updates the command and the display, saves the data and feeds the watchdog.
        """
        motor = MockController(enable_watchdog=True, external_watchdog=True)
        stopping_ramp_instruction = 30.0
        # TODO: zero_position_calibration

        while self.run.value:
            # motor.axis.watchdog_feed()
            control_mode = motor.get_control_mode()
            is_cadence_control = False

            if self.zero_position.value:
                motor.zero_position_calibration()
                self.zero_position.value = False

            if not self.stopping.value:
                # Adapt the control of the motor accordingly to the current cadence and torque
                try:
                    control_mode, direction = self.queue_instructions.get_nowait()
                    motor.set_direction(direction)
                except Exception:
                    pass

                # If the motor is in torque control, the torque input needs to be updated in function of the cadence
                # because of the resisting torque.
                # Furthermore, it allows to stop the pedals by reducing the torque if the user has stopped.
                if control_mode == ControlMode.TORQUE_CONTROL:
                    self.instruction.value = motor.torque_control(self.spin_box.value, self.ramp_instruction.value)

                # The concentric power control mode is based on the torque control mode, but the torque input is
                # calculated from the current cadence (torque_input = f(power / cadence, resiting torque)).
                elif control_mode == ControlMode.CONCENTRIC_POWER_CONTROL:
                    self.instruction.value = motor.concentric_power_control(
                        self.spin_box.value, self.ramp_instruction.value
                    )

                # The linear control mode is based on the torque control mode, but the torque input is calculated from
                # the current cadence (torque_input = linear_coeff * cadence and resiting torque).
                elif control_mode == ControlMode.LINEAR_CONTROL:
                    self.instruction.value = motor.linear_control(self.spin_box.value, self.ramp_instruction.value)

                # The concentric power control mode is based on the cadence control mode, but the cadence input is
                # calculated from the current torque (cadence_input = f(power / torque, resiting torque)).
                elif control_mode == ControlMode.ECCENTRIC_POWER_CONTROL:
                    self.instruction.value = motor.eccentric_power_control(
                        self.spin_box.value, self.ramp_instruction.value
                    )

                # The cadence control mode doesn't need to be adapted at each loop, only once at the change of control
                # type.
                elif control_mode == ControlMode.CADENCE_CONTROL and not is_cadence_control:
                    motor.cadence_control(self.spin_box.value, self.ramp_instruction.value)
                    is_cadence_control = True

            else:
                motor.stopping(cadence_ramp_rate=stopping_ramp_instruction)
                if abs(motor.get_cadence()) < 10.0:
                    self.stopping.value = not motor.stopped()
                is_cadence_control = False

            self.i_measured.value = motor.get_iq_measured()
            self.turns.value = motor.get_turns()
            self.vel_estimate.value = motor.get_vel_estimate()

    def gui_process(self):
        """ """
        app = QtWidgets.QApplication(sys.argv)
        gui = ErgocycleGUI(
            self.run,
            self.instruction,
            self.ramp_instruction,
            self.spin_box,
            self.stopping,
            self.saving,
            self.queue_comment,
            self.i_measured,
            self.turns,
            self.vel_estimate,
            self.queue_instructions,
            self.zero_position,
        )
        gui.show()
        app.exec()
