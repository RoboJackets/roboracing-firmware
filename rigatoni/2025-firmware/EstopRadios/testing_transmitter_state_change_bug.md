# The bug
Occasionally the E-stop remote will switch to the red (disabled) state without anyone pressing the button. We strongly suspect that something is causing the remote to reset, but this only occurs when the car is responding to the remote.

# Observations and tests
 * The E-stop always switches to the red state, never yellow or green.
 * When testing in the shop, this bug occurs roughly every 10 minutes when the car is on.
 * This bug occurs even when the e-stop is plugged in and charging
 * This bug occurs with nobody touching the e-stop

## Test: pin-swapping
We swapped the pins of the yellow (limited) and red (disabled) buttons. It still changed state to red (disabled).

## Test: running overnight
We took the remote home and ran it for ~16 hours. It did NOT change state.

## Test: Defaults to limited mode instead of stop on power-up
The remote then changed state to Limited instead of Stop. Additionally, we saw in a video that all 3 remote lights briefly illuminated, indicating that the remote had reset.

## Test: Running without connection to car at SCC
Remote showed the 4-led flash at random intervals, but in some circumstances as short as a few seconds. Unsure of battery state of charge.

## =================== Updated arduino to 1.8.13, Updated radio library, removed serial prints, extended watchdog timeout =============
