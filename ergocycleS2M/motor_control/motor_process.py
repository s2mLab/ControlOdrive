import multiprocessing as mp
import os
import sys
import time
from ctypes import c_bool, c_double, c_long, c_wchar_p
from PyQt5 import QtWidgets

from ergocycleS2M.data_processing.save import save_data_to_file
from ergocycleS2M.gui.gui import ErgocycleGUI
from ergocycleS2M.motor_control.enums import ControlMode

# from ergocycleS2M.motor_control.self.motor_controller import MotorController
from ergocycleS2M.motor_control.mock_controller import MockController


class MotorProcess():
    def __init__(
            self,
            run: mp.Value,
            zero_position: mp.Value,
            stopping: mp.Value,
            queue_instructions: mp.Queue,
            instruction: mp.Value,
            ramp_instruction: mp.Value,
            spin_box: mp.Value,
            i_measured: mp.Value,
            turns: mp.Value,
            vel_estimate: mp.Value,
            error: mp.Value,
            axis_error: mp.Value,
            controller_error: mp.Value,
            encoder_error: mp.Value,
            motor_error: mp.Value,
            sensorless_estimator_error: mp.Value,
            can_error: mp.Value,
            control_mode: mp.Value,
            direction: mp.Value,
            state: mp.Value,
    ):
        self.run = run
        self.zero_position = zero_position
        self.stopping = stopping
        self.queue_instructions = queue_instructions
        self.instruction = instruction
        self.ramp_instruction = ramp_instruction
        self.spin_box = spin_box
        self.i_measured = i_measured
        self.turns = turns
        self.vel_estimate = vel_estimate
        self.error = error
        self.axis_error = axis_error
        self.controller_error = controller_error
        self.encoder_error = encoder_error
        self.motor_error = motor_error
        self.sensorless_estimator_error = sensorless_estimator_error
        self.can_error = can_error
        self.control_mode = control_mode
        self.direction = direction
        self.state = state


        self.motor = MockController(enable_watchdog=True, external_watchdog=False)

    def motor_control_process(self):
        """
        Main loop of the thread. It is called when the thread is started. It is stopped when the thread is stopped.
        It updates the command and the display, saves the data and feeds the watchdog.
        """

        stopping_ramp_instruction = 30.0
        # TODO: zero_position_calibration
        is_cadence_control = False

        while self.run.value:
            control_mode = self.motor.get_control_mode()

            if self.zero_position.value:
                self.motor.zero_position_calibration()
                self.zero_position.value = False

            if not self.stopping.value:
                # Adapt the control of the self.motor accordingly to the current cadence and torque
                try:
                    control_mode, direction = self.queue_instructions.get_nowait()
                    self.motor.set_direction(direction)
                    is_cadence_control = False
                except Exception:
                    pass

                # If the self.motor is in torque control, the torque input needs to be updated in function of the cadence
                # because of the resisting torque.
                # Furthermore, it allows to stop the pedals by reducing the torque if the user has stopped.
                if control_mode == ControlMode.TORQUE_CONTROL:
                    self.instruction.value = self.motor.torque_control(self.spin_box.value, self.ramp_instruction.value)

                # The concentric power control mode is based on the torque control mode, but the torque input is
                # calculated from the current cadence (torque_input = f(power / cadence, resiting torque)).
                elif control_mode == ControlMode.CONCENTRIC_POWER_CONTROL:
                    self.instruction.value = self.motor.concentric_power_control(
                        self.spin_box.value, self.ramp_instruction.value
                    )

                # The linear control mode is based on the torque control mode, but the torque input is calculated from
                # the current cadence (torque_input = linear_coeff * cadence and resiting torque).
                elif control_mode == ControlMode.LINEAR_CONTROL:
                    self.instruction.value = self.motor.linear_control(self.spin_box.value, self.ramp_instruction.value)

                # The concentric power control mode is based on the cadence control mode, but the cadence input is
                # calculated from the current torque (cadence_input = f(power / torque, resiting torque)).
                elif control_mode == ControlMode.ECCENTRIC_POWER_CONTROL:
                    self.instruction.value = self.motor.eccentric_power_control(
                        self.spin_box.value, self.ramp_instruction.value
                    )

                # The cadence control mode doesn't need to be adapted at each loop, only once at the change of control
                # type.
                elif control_mode == ControlMode.CADENCE_CONTROL and not is_cadence_control:
                    self.motor.cadence_control(self.spin_box.value, self.ramp_instruction.value)
                    is_cadence_control = True

            else:
                self.motor.stopping(cadence_ramp_rate=stopping_ramp_instruction)
                if abs(self.motor.get_cadence()) < 10.0:
                    self.stopping.value = not self.motor.stopped()

            self.i_measured.value = self.motor.get_iq_measured()
            self.turns.value = self.motor.get_turns()
            self.vel_estimate.value = self.motor.get_vel_estimate()
            self.error.value = self.motor.get_error(),
            self.axis_error.value = self.motor.get_axis_error(),
            self.controller_error.value = self.motor.get_controller_error(),
            self.encoder_error.value = self.motor.get_encoder_error(),
            self.motor_error.value = self.motor.get_motor_error(),
            self.sensorless_estimator_error.value = self.motor.get_sensorless_estimator_error(),
            self.can_error.value = self.motor.get_can_error(),
            self.control_mode.value = self.motor.get_control_mode()
            self.direction.value = self.motor.get_direction()
            self.state.value = self.motor.get_state()
