# Flight modes

## Manual

Thottle applies to all the motors equally.

Positive elevation rotates all servos downwards.

Positive pitch causes front up-down servos to rotate upwards, and back ones - downwards.

Positive roll causes left up-down servos to rotate downwards, and right ones - upwards.

Positive yaw causes left motors to increase speed, and right ones to decrease. It also rotates front left/right servos to rotate clockwise, and back ones - counterclockwise. We should check if it will be possible - that is, will it collide with construction, or is it out of the servos' ranges.

## Atti

We will compute main desired force vector (MDFV).

Positive throttle will add to forward component.

Positive sideways will add to rightern component.

Positive elevation will add to upper component.

Based on orientation detected by sensors (especially accelerometer, maybe also gyroscope), we will correct pitch and roll to stay horizontal.

Yaw will add up to desired yaw (persistent between steps/iterations). We'll try to achieve this desired yaw.

For pitch, roll and yaw stabilization, we'll likely use PID regulators.

For each motor, we'll compute desired force vector. It will be the sum of the main desired force vector (MDFV) and local rotating force vector (LRFV).

Positive computed pitch will subtract from front LRFVs' up components, and add to back ones' up components.

Positive computed roll will add to left LRFV's up components, and subtract from right ones' up components.

Positive computed yaw will add that motor's displacement (position relative to the center of mass) rotated clockwise to the LRFV.

Then, for every motor, we'll compute servo angles based on its LRFV's direction, and set motor speed to its magnitude.

## AltiAtti

Very similar to Atti, but we'll instead add elevation to persistent desired altitude, and use PID to try to achieve that altitude.

Computed computed elevation will add to MDFV's up component.

## Future flight modes

GPS - position stabilization, so wind will automatically be countered.

Waypoints - fly to set points.
