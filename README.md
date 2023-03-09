# ControlOdrive
I'm not sure `odrive` can be used on the last version of Python, so I have not updated it from ``
## Hardware
### Relays
The relays are not controlled by the Odrive, they are there to ensure that when the power source that powers the Odrive is sut down, the current cannot go into it.
As soon as the Odrive is powered, the relays are too and the motor is directly controlled by the Odrive.
To work, if the Odrive is connected to the COM pins and the motor to the NO pins,
the COM pins need to be connected to the High pins (so the normally open relay closes when alimented).
### Reduction ratio
There are two gear stages. The first one has 8 teeth at the entry and 36 (motor) at the exit (blue gear).
The second one has supposedly (according to my researches and check on the hardware) 10 teeth at the entry and 91 at the exit. 
The reduction ratio is then Ze/Zs = ws/we = 8/36*10/91
## Firmware/Software
2022/02/09:
- Firmware: 0.5.5, *a priori* the last firmware available for the Odrive v3.6 56V (end of life)
  - /!\ I updated the firmware od only one board
  - To update the firmware on the Raspberry, use `sudo odrivetool dfu path/to/firmware/file.elf`
- Software: odrive 0.6.3.post0, the latest version at this time
  - To update the software on the Raspberry, use `pip install odrive==0.6.3.post0`
## Protocol
### Torque constant and resisting torque
#### Resisting torque
The first step is to determine the resisting torque of the motr. The resisting torque is the minimum torque that the
motor has to apply to make the pedals move.
To do so, you must use the torque control on the system with the neutral load (just the pedals, not any additional
load). The torque profile must be a ramp and the torque at which the motor is moving is the resisting torque. You 
should do it several times in order to have a precise value. It is advised to do it for each rotation direction and with
different `torqu_ramps`.
At this point, we don't know the torque constant, so we can only have the corresponding current.
The script `find_resisting_torque/resisting/torque.py` allows to compute the current corresponding to the resisting
torque. _A priori_ the motor won't turn if it as less current than this `resisting_current`.
#### Torque constant
Supposedly, 
$$
\Tau_{motor} = K_I * `Iq_measured`
\Tau_{out} = \mu \Tau_motor / `reduction_ratio`
$$
## Troubleshooting
### Calibration
Every calibration must be done without mechanical load nor watchdog since the motor has to reboot several times.
#### Complete calibration
The complete calibration must be done without any mechanical load.
### Watchdog
The watchdog timeout can't be as little as the user wants.
For instance, for a feeding time of `0.01`, the watchdog can't be as small as `0.05`.
I've set it to `0.1`.
### Connection to the Odrive
I had trouble connecting the Odrive to the Raspberry Pi. The solution that worked for me was to write in a terminal:
```
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="0d[0-9][0-9]", MODE="0666"' | sudo tee /etc/udev/rules.d/91-odrive.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```
To test the connection a simple script is sufficient :
```
import odrive

print("Look for an odrive ...")
odrive.find_any()
print("Found")
```
Or when launching `odrivetool` in a terminal, it should be written :
```
ODrive control utility v0.5.4
Connected to ODrive 206F3694424D as odrv0
```
/!\ If a script using the connection to the Odrive is running, you cannot connect to the Odrive through `odrivetool`
### Motor is not moving at all
The simple test that should work before trying anything else is :
```
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
```
I had trouble controlling the motor with the relay. When connecting it directly to the Odrive it worked.
### Errors about the current, the resistance, the inductance
Verify you have :
`resistance_calib_max_voltage > calibration_current * phase_resistance`
`resistance_calib_max_voltage < 0.5 * vbus_voltage`

For me (TSDZ2 connected directly to the Odrive), it worked with the following config :
```
I_bus_hard_max: inf (float)
I_bus_hard_min: -inf (float)
I_leak_max: 0.10000000149011612 (float)
R_wL_FF_enable: False (bool)
acim_autoflux_attack_gain: 10.0 (float)
acim_autoflux_decay_gain: 1.0 (float)
acim_autoflux_enable: False (bool)
acim_autoflux_min_Id: 10.0 (float)
acim_gain_min_flux: 10.0 (float)
bEMF_FF_enable: False (bool)
calibration_current: 30.0 (float)
current_control_bandwidth: 100.0 (float)
current_lim: 35.0 (float)
current_lim_margin: 8.0 (float)
dc_calib_tau: 0.20000000298023224 (float)
inverter_temp_limit_lower: 100.0 (float)
inverter_temp_limit_upper: 120.0 (float)
motor_type: 0 (uint8)
phase_inductance: 2.409255648672115e-05 (float)
phase_resistance: 0.04511798173189163 (float)
pole_pairs: 8 (int32)
pre_calibrated: True (bool)
requested_current_range: 25.0 (float)
resistance_calib_max_voltage: 10.0 (float)
torque_constant: 0.20999999344348907 (float)
torque_lim: inf (float)
```
### Position control and velocity control
Setting the `odrv0.axis0.controller.config.control_mode`, the `odrv0.axis0.controller.config.input_mode` and the 
`odrv0.axis0.requested_state` must be done only one and not between each change of position or velocity !