package org.firstinspires.ftc.teamcode.Utils;

import com.pedropathing.geometry.Pose;

/**
 * Utility helpers for working with {@link Pose} objects.
 */
public class PoseUtils {
	/**
	 * Mirror a pose across the vertical axis of the field at {@code x = fieldLength / 2}.
	 * <p>
	 * For a field of length {@code fieldLength}, the transform is:
	 * <pre>
	 *   (x, y, θ)  →  (fieldLength − x, y, −θ)
	 * </pre>
	 * Y is unchanged because mirroring is across the vertical center line of the field.
	 * Heading is negated because the robot faces the opposite direction after the flip.
	 *
	 * @param pose        the source pose to mirror
	 * @param fieldLength the length of the field along the axis being mirrored (in inches)
	 * @return a new {@link Pose} that is the mirrored copy
	 */
	public static Pose mirror(Pose pose, double fieldLength) {
		return new Pose(
				fieldLength - pose.getX(),
				pose.getY(),
				-pose.getHeading()
		);
	}
}
