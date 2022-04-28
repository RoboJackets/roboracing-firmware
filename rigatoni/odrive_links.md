**Getting started:** https://docs.odriverobotics.com/v/latest/getting-started.html

Odrivetool: https://docs.odriverobotics.com/v/latest/odrivetool.html

States and startup: https://docs.odriverobotics.com/v/latest/commands.html
Endstops: https://docs.odriverobotics.com/v/latest/endstops.html
 * The endstop guide seems to be wrong: you need to set odrv0.axis0.min_endstop.config.offset > 0.
 * Switch is between GND and GPIO 5

Odrive doesn't respond to position setpoints: https://discourse.odriverobotics.com/t/my-motor-wont-move-without-any-errors/6079
 * `odrv0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL`
 * `odrv0.axis0.controller.config.input_mode = INPUT_MODE_PASSTHROUGH`

Electrical noise mitigation: https://discourse.odriverobotics.com/t/encoder-error-error-illegal-hall-state/1047/8

## Starting up the brakes
Be about 7 turns or more away from the endstop before running this so the motor doesn't hit the endstop.
 1. `odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE`
 2. `odrv0.axis0.requested_state = AXIS_STATE_HOMING`
 3. `odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL`

Then `odrv0.axis0.controller.input_pos = 1` to set any position you want.