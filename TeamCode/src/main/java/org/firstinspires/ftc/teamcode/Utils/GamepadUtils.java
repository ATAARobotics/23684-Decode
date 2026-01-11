package org.firstinspires.ftc.teamcode.Utils;

public class GamepadUtils {

	/**
	 * Returns true if the value is within the specified deadzone.
	 * Use this for simple boolean checks.
	 */
	public static boolean inDeadzone(double value, double threshold) {
		return Math.abs(value) < threshold;
	}

	/**
	 * Overloaded version with a default 0.1 threshold.
	 */
	public static boolean inDeadzone(double value) {
		return inDeadzone(value, 0.1);
	}

	/**
	 * Filters the input: returns 0.0 if within the deadzone,
	 * otherwise returns the original value.
	 */
	public static double applyDeadzone(double value, double threshold) {
		return Math.abs(value) < threshold ? 0.0 : value;
	}

	/**
	 * Overloaded version with a default 0.1 threshold.
	 */
	public static double applyDeadzone(double value) {
		return Math.abs(value) < 0.1 ? 0.0 : value;
	}
}