"""
This module contains the main application class. This application manages the gui, the motor and the saving in different
processes with a shared memory.
"""
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
    """
    This class is the main application. It creates launch the gui, the motor and the saving in different processes with
    a shared memory.

    Attributes
    ----------
    saving_frequency : float
        Frequency of the saving in Hz.
    run : mp.Value
        Shared memory value to stop the application.
    zero_position : mp.Value
        Shared memory value to indicate to set the zero position of the motor.
    queue_instructions : mp.Queue
        Shared memory queue to send instructions to the motor.
    training_mode : mp.Value
        Shared memory value to indicate the current training mode.
    spin_box : mp.Value
        Shared memory value to indicate the current spin box value.
    instruction : mp.Value
        Shared memory value to indicate the current instruction.
    ramp_instruction : mp.Value
        Shared memory value to indicate the current ramp instruction.
    stopping : mp.Value
        Shared memory value to indicate if the motor is stopping.
    saving : mp.Value
        Shared memory value to indicate to save the data in a file.
    queue_file_name : mp.Queue
        Shared memory queue to send the file name to the saving process.
    queue_comment : mp.Queue
        Shared memory queue to send the comment to the saving process.
    stopwatch : mp.Value
        Shared memory value to indicate the current stopwatch value.
    lap : mp.Value
        Shared memory value to indicate the current lap value.
    i_measured : mp.Value
        Shared memory value to indicate the current measured current in A.
    turns : mp.Value
        Shared memory value to indicate the current number of turns in tr.
    vel_estimate : mp.Value
        Shared memory value to indicate the current estimated velocity at the motor in tr/s.
    error : mp.Value
        Shared memory value to indicate the current odrv.error.
    axis_error : mp.Value
        Shared memory value to indicate the current odrv.axis.error.
    controller_error : mp.Value
        Shared memory value to indicate the current odrv.axis.controller.error.
    encoder_error : mp.Value
        Shared memory value to indicate the current odrv.axis.encoder.error.
    motor_error : mp.Value
        Shared memory value to indicate the current odrv.axis.motor.error.
    sensorless_estimator_error : mp.Value
        Shared memory value to indicate the current odrv.axis.sensorless_estimator.error.
    can_error : mp.Value
        Shared memory value to indicate the current odrv.can.error.
    gui_process : mp.Process
        Process of the gui.
    saving_process : mp.Process
        Process of the motor control.

    Methods
    -------
    start
        Start the application.
    motor_control_loop
        Loop of the motor control it updates the control accordingly to the information received from the gui process
        and update its data in the shared memory.
        To be launched in the main process.
    gui_loop
        Loop of the gui it updates the data accordingly to the information received from the motor control process and
        update the control of the motor in the shared memory.
        To be launched in a parallel process.
    saving_loop
        Loop of the saving process it saves the data in a file.
        To be launched in a parallel process.
    """

    def __init__(self, saving_frequency: float = 10):
        self.saving_frequency = saving_frequency

        # Shared memory
        # Security
        self.run = mp.Manager().Value(c_bool, True)
        # Control
        self.zero_position = mp.Manager().Value(c_bool, 0.0)
        self.queue_instructions = mp.Manager().Queue()
        self.training_mode = mp.Manager().Value(c_wchar_p, None)
        self.control_mode = mp.Manager().Value(c_wchar_p, None)
        self.direction = mp.Manager().Value(c_wchar_p, None)
        self.spin_box = mp.Manager().Value(c_double, 0.0)
        self.instruction = mp.Manager().Value(c_double, 0.0)
        self.ramp_instruction = mp.Manager().Value(c_double, 0.0)
        self.stopping = mp.Manager().Value(c_bool, False)
        # Saving
        self.saving = mp.Manager().Value(c_bool, False)
        self.queue_file_name = mp.Manager().Queue()
        self.queue_comment = mp.Manager().Queue()
        # Stopwatch
        self.stopwatch = mp.Manager().Value(c_double, 0.0)
        self.lap = mp.Manager().Value(c_double, 0.0)
        # Data
        self.i_measured = mp.Manager().Value(c_double, 0.0)
        self.turns = mp.Manager().Value(c_double, 0.0)
        self.vel_estimate = mp.Manager().Value(c_double, 0.0)
        self.state = mp.Manager().Value(c_long, None)
        # Errors
        self.error = mp.Manager().Value(c_long, 0)
        self.axis_error = mp.Manager().Value(c_long, 0)
        self.controller_error = mp.Manager().Value(c_long, 0)
        self.encoder_error = mp.Manager().Value(c_long, 0)
        self.motor_error = mp.Manager().Value(c_long, 0)
        self.sensorless_estimator_error = mp.Manager().Value(c_long, 0)
        self.can_error = mp.Manager().Value(c_long, 0)

        # Create the processes
        self.gui_process = mp.Process(name="GUI process", target=self.gui, daemon=True)
        self.saving_process = mp.Process(name="Saving process", target=self.saving_loop, daemon=True)

    def start(self):
        """
        Start the application.
        """
        self.gui_process.start()
        self.saving_process.start()
        self.motor_control_loop()
        # We join the processes to be sure that they are stopped before exiting the application.
        self.gui_process.join()
        self.saving_process.join()

    def motor_control_loop(self):
        """
        Loop of the motor control it updates the control accordingly to the information received from the gui process
        and update its data in the shared memory.
        To be launched in the main process.
        """
        motor = MotorController(enable_watchdog=True, external_watchdog=False)
        motor.zero_position_calibration()
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
                motor.stopping(cadence_ramp_rate=self.ramp_instruction.value)
                if abs(motor.get_cadence()) < 10.0:
                    self.stopping.value = not motor.stopped()

            # Data
            self.i_measured.value = motor.get_iq_measured()
            self.turns.value = motor.get_turns()
            self.vel_estimate.value = motor.get_vel_estimate()
            self.control_mode.value = motor.get_control_mode()
            self.direction.value = motor.get_direction()
            self.state.value = motor.get_state()
            # Errors
            self.error.value = (motor.get_error(),)
            self.axis_error.value = (motor.get_axis_error(),)
            self.controller_error.value = (motor.get_controller_error(),)
            self.encoder_error.value = (motor.get_encoder_error(),)
            self.motor_error.value = (motor.get_motor_error(),)
            self.sensorless_estimator_error.value = (motor.get_sensorless_estimator_error(),)
            self.can_error.value = (motor.get_can_error(),)

    def gui(self):
        """
        Loop of the gui it updates the data accordingly to the information received from the motor control process and
        update the control of the motor in the shared memory.
        """
        app = QtWidgets.QApplication(sys.argv)
        gui = ErgocycleGUI(
            run=self.run,
            zero_position=self.zero_position,
            queue_instructions=self.queue_instructions,
            training_mode=self.training_mode,
            spin_box=self.spin_box,
            instruction=self.instruction,
            ramp_instruction=self.ramp_instruction,
            stopping=self.stopping,
            saving=self.saving,
            queue_file_name=self.queue_file_name,
            queue_comment=self.queue_comment,
            stopwatch=self.stopwatch,
            lap=self.lap,
            i_measured=self.i_measured,
            turns=self.turns,
            vel_estimate=self.vel_estimate,
            error=self.error,
            axis_error=self.axis_error,
            controller_error=self.controller_error,
            encoder_error=self.encoder_error,
            motor_error=self.motor_error,
            sensorless_estimator_error=self.sensorless_estimator_error,
            can_error=self.can_error,
        )
        gui.show()
        app.exec()

    def saving_loop(self):
        """
        Loop of the saving process it saves the data in a file.
        """
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

                previous_save_time = saving_start_time = time.time()
                # TODO: Save at a fixed frequency
                while self.run.value and self.saving.value:
                    try:
                        comment = self.queue_comment.get_nowait()
                    except Exception:
                        comment = ""
                    # Force the saving to be done at the specified frequency
                    while time.time() - previous_save_time < 1.0 / self.saving_frequency:
                        pass
                    save_data_to_file(
                        file_path=file_name,
                        time=time.time() - saving_start_time,
                        spin_box=self.spin_box.value,
                        instruction=self.instruction.value,
                        ramp_instruction=self.ramp_instruction.value,
                        comment=comment,
                        stopwatch=self.stopwatch.value,
                        lap=self.lap.value,
                        state=self.state.value,
                        training_mode=self.training_mode.value,
                        control_mode=self.control_mode.value,
                        direction=self.direction.value,
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
                    previous_save_time = time.time()
            except Exception:
                # No file name in the queue meaning that no saving instruction has been sent.
                pass
