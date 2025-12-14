package org.firstinspires.ftc.teamcode.Utilities;

public class UnwrapAngle {

	/**
	 * Calculates the next continuous angle based on the previous state.
	 *
	 * @param previousRaw       The raw angle from the previous step (e.g., 359).
	 * @param currentRaw        The raw angle from the current sensor reading (e.g., 1).
	 * @param previousUnwrapped The continuous, accumulated angle calculated in the previous step.
	 * @return The new continuous angle (e.g., 361).
	 */
	public static double unwrap(double previousRaw, double currentRaw, double previousUnwrapped) {
		// 1. Calculate the raw difference
		double diff = currentRaw - previousRaw;

		// 2. Normalize the difference to be between -180 and 180
		// If the jump is too big, it means we crossed the 0/360 boundary
		if (diff > 90) {
			diff -= 180; // We crossed from high to low (e.g., 0 -> 350 is -10, not +350)
		} else if (diff < -90) {
			diff += 180; // We crossed from low to high (e.g., 350 -> 0 is +10, not -350)
		}

		// 3. Add the corrected difference to the running total
		return previousUnwrapped + diff;
	}
}
