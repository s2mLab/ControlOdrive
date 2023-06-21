import multiprocessing as mp
import os
import sys
import time
from ctypes import c_bool, c_double, c_long, c_wchar_p
from PyQt5 import QtWidgets

from ergocycleS2M.data_processing.save import save_data_to_file
from ergocycleS2M.gui.gui import ErgocycleGUI
from ergocycleS2M.motor_control.enums import ControlMode

from ergocycleS2M.motor_control.motor_controller import MotorController
# from ergocycleS2M.motor_control.mock_controller import MockController


class Application:
    """ """

    def __init__(self, saving_frequency: float = 10):
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
        self.queue_file_name = mp.Manager().Queue()
        self.queue_comment = mp.Manager().Queue()
        self.saving_frequency = saving_frequency
        self.error = mp.Manager().Value(c_long, 0)
        self.axis_error = mp.Manager().Value(c_long, 0)
        self.controller_error = mp.Manager().Value(c_long, 0)
        self.encoder_error = mp.Manager().Value(c_long, 0)
        self.motor_error = mp.Manager().Value(c_long, 0)
        self.sensorless_estimator_error = mp.Manager().Value(c_long, 0)
        self.can_error = mp.Manager().Value(c_long, 0)
        self.control_mode = mp.Manager().Value(c_wchar_p, None)
        self.direction = mp.Manager().Value(c_wchar_p, None)
        self.stopwatch = mp.Manager().Value(c_double, 0.0)
        self.lap = mp.Manager().Value(c_double, 0.0)
        self.training_mode = mp.Manager().Value(c_wchar_p, None)
        self.state = mp.Manager().Value(c_long, None)

        # Create the processes
        self.motor_process = mp.Process(name="motor", target=self.motor_control_process, daemon=True)
        self.gui_process = mp.Process(name="gui", target=self.gui_fun_process, daemon=True)
        self.save_process = mp.Process(name="save", target=self.save_fun_process, daemon=True)

    def start(self):
        """ """
        # self.motor_process.start()
        self.gui_process.start()
        self.save_process.start()
        self.motor_control_process()

    def motor_control_process(self):
        """
        Main loop of the thread. It is called when the thread is started. It is stopped when the thread is stopped.
        It updates the command and the display, saves the data and feeds the watchdog.
        """
        motor = MotorController(enable_watchdog=True, external_watchdog=False)
        stopping_ramp_instruction = 30.0
        # TODO: zero_position_calibration
        is_cadence_control = False

        while self.run.value:
            control_mode = motor.get_control_mode()

            if self.zero_position.value:
                motor.zero_position_calibration()
                self.zero_position.value = False

            if not self.stopping.value:
                # Adapt the control of the motor accordingly to the current cadence and torque
                try:
                    control_mode, direction = self.queue_instructions.get_nowait()
                    motor.set_direction(direction)
                    is_cadence_control = False
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

            self.i_measured.value = motor.get_iq_measured()
            self.turns.value = motor.get_turns()
            self.vel_estimate.value = motor.get_vel_estimate()
            self.error.value = motor.get_error(),
            self.axis_error.value = motor.get_axis_error(),
            self.controller_error.value = motor.get_controller_error(),
            self.encoder_error.value = motor.get_encoder_error(),
            self.motor_error.value = motor.get_motor_error(),
            self.sensorless_estimator_error.value = motor.get_sensorless_estimator_error(),
            self.can_error.value = motor.get_can_error(),
            self.control_mode.value = motor.get_control_mode()
            self.direction.value = motor.get_direction()
            self.state.value = motor.get_state()

    def gui_fun_process(self):
        """ """
        app = QtWidgets.QApplication(sys.argv)
        gui = ErgocycleGUI(
            self.run,
            self.instruction,
            self.ramp_instruction,
            self.spin_box,
            self.stopping,
            self.saving,
            self.queue_file_name,
            self.queue_comment,
            self.i_measured,
            self.turns,
            self.vel_estimate,
            self.queue_instructions,
            self.zero_position,
            self.stopwatch,
            self.lap,
            self.training_mode,
            self.error,
            self.axis_error,
            self.controller_error,
            self.encoder_error,
            self.motor_error,
            self.sensorless_estimator_error,
            self.can_error,
        )
        gui.show()
        app.exec()

    def save_fun_process(self):
        """ """
        while self.run.value:
            try:
                file_name = self.queue_file_name.get_nowait()
                # Choosing the file name at the beginning of the saving.
                ext = ".bio"
                if os.path.isfile(f"{file_name}{ext}"):
                    # File already exists, add a suffix to the filename
                    i = 1
                    while os.path.isfile(f"{file_name}_{i}{ext}"):
                        i += 1
                    file_name = f"{file_name}_{i}"

                time_prev_save = t0 = time.time()
                while self.saving.value:
                    try:
                        comment = self.queue_comment.get_nowait()
                    except Exception:
                        comment = ""
                    save_data_to_file(
                        file_path=file_name,
                        time=time.time() - t0,
                        spin_box=self.spin_box.value,
                        instruction=self.instruction.value,
                        ramp_instruction=self.ramp_instruction.value,
                        comment=comment,
                        stopwatch=self.stopwatch.value,
                        lap=self.lap.value,
                        state=self.state.value,
                        control_mode=self.control_mode.value,
                        direction=self.direction.value,
                        training_mode=self.training_mode.value,
                        vel_estimate=self.vel_estimate.value,
                        turns=self.turns.value,
                        iq_measured=self.i_measured.value,
                        error=self.error.value[0],
                        axis_error=self.axis_error.value[0],
                        controller_error=self.controller_error.value[0],
                        encoder_error=self.encoder_error.value[0],
                        motor_error=self.motor_error.value[0],
                        sensorless_estimator_error=self.sensorless_estimator_error.value[0],
                        can_error=self.can_error.value[0],
                    )
                    loop_time = time.time() - time_prev_save
                    time_prev_save = time.time()
                    if loop_time < 1.0 / self.saving_frequency:
                        time.sleep(1.0 / self.saving_frequency - loop_time)
            except Exception:
                pass
