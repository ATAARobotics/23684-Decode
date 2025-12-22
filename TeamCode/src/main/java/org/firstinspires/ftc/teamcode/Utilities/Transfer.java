package org.firstinspires.ftc.teamcode.Utilities;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;

/**
 * Utility class for checking if the transfer system is ready to shoot.
 * Combines checks for:
 * - Spindexer position within tolerance of a shooting position
 * - Shooter RPM within tolerance of target RPM
 */
public class Transfer {
	// Spindexer tolerance
	public static final double SPINDEXER_TOLERANCE_DEGREES = 10.0;
	
	// Shooting positions (in degrees)
	private static final double[] SHOOTING_POSITIONS = {20.0, 140.0, 260.0};
	
	/**
	 * Checks if the spindexer is within tolerance of any shooting position.
	 *
	 * @param spindexer The spindexer subsystem instance
	 * @return true if the spindexer is at a shooting position (within tolerance), false otherwise
	 */
	public static boolean isSpindexerAtShootingPosition(Spindexer spindexer) {
		// TODO: FIX
//		for (double shootingPos : SHOOTING_POSITIONS) {
//			if (isWithinTolerance(currentPosition, shootingPos, SPINDEXER_TOLERANCE_DEGREES)) {
//				return true;
//			}
//		}
		return false;
	}
	
	/**
	 * Checks if the shooter is at the target RPM (within tolerance).
	 *
	 * @param shooter The shooter subsystem instance
	 * @param targetRPM The target RPM to check against
	 * @return true if the shooter RPM is within tolerance of the target, false otherwise
	 */
	public static boolean isShooterAtTargetRPM(Shooter shooter, double targetRPM) {
		return shooter.isAtTargetRPM(targetRPM);
	}
	
	/**
	 * Checks if both the spindexer and shooter are ready for a transfer (shot).
	 * Both conditions must be met:
	 * - Spindexer is at a shooting position (within 10 degrees tolerance)
	 * - Shooter is at the target RPM (within RPM_TOLERANCE)
	 *
	 * @param spindexer The spindexer subsystem instance
	 * @param shooter The shooter subsystem instance
	 * @param targetRPM The target RPM for the shooter
	 * @return true if both conditions are met, false otherwise
	 */
	public static boolean isTransferReady(Spindexer spindexer, Shooter shooter, double targetRPM) {
		return isSpindexerAtShootingPosition(spindexer) && isShooterAtTargetRPM(shooter, targetRPM);
	}
	
	/**
	 * Helper method to check if a value is within tolerance of a target.
	 * Accounts for circular wraparound (0° = 360°).
	 *
	 * @param current The current value
	 * @param target The target value
	 * @param tolerance The allowed tolerance
	 * @return true if current is within tolerance of target, false otherwise
	 */
	private static boolean isWithinTolerance(double current, double target, double tolerance) {
		// Normalize both values to 0-360 range
		current = normalizeAngle(current);
		target = normalizeAngle(target);
		
		// Calculate the shortest angular distance
		double diff = Math.abs(current - target);
		diff = Math.min(diff, 360.0 - diff);
		
		return diff <= tolerance;
	}
	
	/**
	 * Normalizes an angle to the 0-360 range.
	 *
	 * @param angle The angle to normalize
	 * @return The normalized angle in the range [0, 360)
	 */
	private static double normalizeAngle(double angle) {
		angle = angle % 360.0;
		if (angle < 0) {
			angle += 360.0;
		}
		return angle;
	}
}
