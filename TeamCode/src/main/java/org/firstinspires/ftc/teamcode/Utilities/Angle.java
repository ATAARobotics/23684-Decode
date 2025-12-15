package org.firstinspires.ftc.teamcode.Utilities;

public class Angle {

	/**
	 * Calculates the next continuous angle based on the previous state.
	 *
	 * @param previousRaw       The raw angle from the previous step (e.g., 359).
	 * @param currentRaw        The raw angle from the current sensor reading (e.g., 1).
	 * @param previousUnwrapped The continuous, accumulated angle calculated in the previous step.
	 * @return The new continuous angle (e.g., 361).
	 */
	public static double unwrap(double previousRaw, double currentRaw, double previousUnwrapped) {
		return unwrap(previousRaw, currentRaw, previousUnwrapped, 360);
	}

	/**
	 * Calculates the next continuous angle based on the previous state.
	 *
	 * @param previousRaw       The raw angle from the previous step (e.g., 359).
	 * @param currentRaw        The raw angle from the current sensor reading (e.g., 1).
	 * @param previousUnwrapped The continuous, accumulated angle calculated in the previous step.
	 * @param range             The wrapping range (e.g., 360 for [0, 360], 180 for [-180, 180]).
	 * @return The new continuous angle (e.g., 361).
	 */
	public static double unwrap(double previousRaw, double currentRaw, double previousUnwrapped, double range) {
		// 1. Calculate the raw difference
		double diff = currentRaw - previousRaw;

		// 2. Normalize the difference to be within half the range
		// If the jump is too big, it means we crossed a boundary
		double threshold = range / 2.0;
		if (diff > threshold) {
			diff -= range; // We crossed from high to low
		} else if (diff < -threshold) {
			diff += range; // We crossed from low to high
		}

		// 3. Add the corrected difference to the running total
		return previousUnwrapped + diff;
	}

	/**
	 * Finds the nearest coterminal (equivalent) angle to the current angle.
	 * For example, if target=90 and current=400, checks 90, 450, 810, -270, etc.
	 * and returns the closest one to current (in this case, 450).
	 *
	 * @param target  The target angle (e.g., 90).
	 * @param current The current angle (e.g., 400).
	 * @return The nearest equivalent angle to current. If tied, returns the larger one.
	 */
	public static double nearestCoterminal(double target, double current) {
		return nearestCoterminal(target, current, 360);
	}

	/**
	 * Finds the nearest coterminal (equivalent) angle to the current angle.
	 *
	 * @param target  The target angle (e.g., 90).
	 * @param current The current angle (e.g., 400).
	 * @param range   The wrapping range (e.g., 360 for [0, 360], 180 for [-180, 180]).
	 * @return The nearest equivalent angle to current. If tied, returns the larger one.
	 */
	public static double nearestCoterminal(double target, double current, double range) {
		double best = target;
		double bestDistance = Math.abs(best - current);

		// Check coterminal angles by adding/subtracting multiples of range
		for (int i = -2; i <= 2; i++) {
			if (i == 0) continue; // Already checked the base target
			double candidate = target + (i * range);
			double distance = Math.abs(candidate - current);

			// Update if closer, or if equal distance but candidate is larger
			if (distance < bestDistance || (distance == bestDistance && candidate > best)) {
				best = candidate;
				bestDistance = distance;
			}
		}

		return best;
	}
}
