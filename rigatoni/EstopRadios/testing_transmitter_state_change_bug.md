# The bug
Occasionally the E-stop remote will switch to the red (disabled) state without anyone pressing the button.

# Observations and tests
 * The E-stop always switches to the red state, never yellow or green.
 * When testing in the shop, this bug occurs roughly every hour or two.
 * This bug occurs even when the e-stop is plugged in and charging
 * This bug occurs with nobody touching the e-stop

## Test: pin-swapping
We swapped the pins of the yellow (limited) and red (disabled) buttons. It still changed state to red (disabled).

## Test: running overnight
We took the remote home and ran it for ~16 hours. It did NOT change state.