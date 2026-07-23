package org.firstinspires.ftc.teamcode.Utils;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;

import org.firstinspires.ftc.teamcode.OpModes.Auto.Modular.PoseDatabase;

/**
 * Field-relative shooting zones used to gate scoring decisions.
 * <p>
 * The two triangles below match the ALLIANCE-SPECIFIC goal regions on the
 * DECODE field, expressed in BLUE alliance coordinates:
 * <ul>
 *   <li>{@link #ZONE_A} — audience-side goal region, base along y = 144</li>
 *   <li>{@link #ZONE_B} — goal-side region, base along y = 0</li>
 * </ul>
 * Both zones are expressed in the BLUE coordinate frame; callers on the RED
 * alliance should mirror the robot pose (see {@link #nearestEntryInRedZone}) before
 * checking, or rely on the alliance-aware wrapper.
 * <p>
 * The footprint constants {@link #ROBOT_FORWARD_EXTENT} and
 * {@link #ROBOT_SIDEWAYS_EXTENT} define the 13.5" × 15.5" rectangle that
 * approximates the robot's chassis. A pose is considered "in zone" when at
 * least one of its four footprint corners lies inside either triangle.
 */
public final class ShootingZone {

	public static final double ROBOT_FORWARD_EXTENT = 13.5;
	public static final double ROBOT_SIDEWAYS_EXTENT = 15.5;

	/** Audience-side goal triangle: (0,144)-(72,72)-(144,144). */
	public static final double[] ZONE_A = {0.0, 144.0, 72.0, 72.0, 144.0, 144.0};
	/** Goal-side triangle: (48,0)-(72,24)-(96,0). */
	public static final double[] ZONE_B = {48.0, 0.0, 72.0, 24.0, 96.0, 0.0};

	private static final double FIELD_LENGTH = PoseDatabase.FIELD_LENGTH;

	private ShootingZone() {}

	/**
	 * True when at least one corner of the robot footprint at {@code pose} lies
	 * inside either BLUE-side shooting zone triangle.
	 */
	public static boolean isAnyCornerInBlueZone(Pose pose) {
		return isAnyCornerInZone(pose, ZONE_A, ZONE_B);
	}

	/**
	 * True when at least one corner of the robot footprint at {@code pose} lies
	 * inside either RED-side shooting zone triangle. Mirrors {@code pose} to the
	 * BLUE frame, tests against {@link #ZONE_A}/{@link #ZONE_B}, and reflects
	 * the result.
	 */
	public static boolean isAnyCornerInRedZone(Pose pose) {
		return isAnyCornerInZone(PoseUtils.mirror(pose, FIELD_LENGTH), ZONE_A, ZONE_B);
	}

	/**
	 * Returns the closest point on the union of the two BLUE shooting-zone
	 * triangles to {@code pose}. If {@code pose} is already inside either
	 * triangle the pose itself is returned (clamped to the triangle).
	 */
	public static Pose nearestEntryInBlueZone(Pose pose) {
		return nearestPointInZone(pose, ZONE_A, ZONE_B);
	}

	/**
	 * Returns the closest point on the union of the two RED shooting-zone
	 * triangles to {@code pose}, then mirrors the result back into the RED
	 * coordinate frame.
	 */
	public static Pose nearestEntryInRedZone(Pose pose) {
		Pose inBlue = nearestPointInZone(
				PoseUtils.mirror(pose, FIELD_LENGTH), ZONE_A, ZONE_B);
		return PoseUtils.mirror(inBlue, FIELD_LENGTH);
	}

	/**
	 * Signed perpendicular distance from {@code pose} to the nearest edge of
	 * either BLUE zone triangle. Positive = inside, negative = outside. Zero
	 * is on the boundary. Used to enforce a "≥1 inch inside" safety margin
	 * before scoring.
	 */
	public static double signedDistanceIntoBlueZone(Pose pose) {
		return signedDistanceIntoZone(pose, ZONE_A, ZONE_B);
	}

	/**
	 * Signed perpendicular distance from {@code pose} (in RED coordinates) to
	 * the nearest edge of either RED zone triangle.
	 */
	public static double signedDistanceIntoRedZone(Pose pose) {
		return signedDistanceIntoZone(
				PoseUtils.mirror(pose, FIELD_LENGTH), ZONE_A, ZONE_B);
	}

	// ----- internal geometry helpers -----

	private static boolean isAnyCornerInZone(Pose pose, double[] triA, double[] triB) {
		double cx = pose.getX();
		double cy = pose.getY();
		double heading = pose.getHeading();

		double halfForward = ROBOT_FORWARD_EXTENT / 2.0;
		double halfSideways = ROBOT_SIDEWAYS_EXTENT / 2.0;

		double cos = Math.cos(heading);
		double sin = Math.sin(heading);

		for (int sx = -1; sx <= 1; sx += 2) {
			for (int sy = -1; sy <= 1; sy += 2) {
				double lx = sx * halfSideways;
				double ly = sy * halfForward;
				double fx = ly * cos - lx * sin;
				double fy = lx * cos + ly * sin;
				Pose corner = new Pose(cx + fx, cy + fy);
				if (isInTriangle(corner, triA) || isInTriangle(corner, triB)) {
					return true;
				}
			}
		}
		return false;
	}

	private static Pose nearestPointInZone(Pose pose, double[] triA, double[] triB) {
		Pose pA = nearestPointInTriangle(pose, triA);
		Pose pB = nearestPointInTriangle(pose, triB);
		double dA = distance(pose, pA);
		double dB = distance(pose, pB);
		return dA <= dB ? pA : pB;
	}

	private static double signedDistanceIntoZone(Pose pose, double[] triA, double[] triB) {
		double dA = signedDistanceToTriangle(pose, triA);
		double dB = signedDistanceToTriangle(pose, triB);
		return Math.max(dA, dB);
	}

	private static boolean isInTriangle(Pose p, double[] tri) {
		double px = p.getX();
		double py = p.getY();
		double ax = tri[0], ay = tri[1];
		double bx = tri[2], by = tri[3];
		double cx = tri[4], cy = tri[5];

		double d1 = sign(px, py, ax, ay, bx, by);
		double d2 = sign(px, py, bx, by, cx, cy);
		double d3 = sign(px, py, cx, cy, ax, ay);

		boolean hasNeg = (d1 < 0) || (d2 < 0) || (d3 < 0);
		boolean hasPos = (d1 > 0) || (d2 > 0) || (d3 > 0);
		return !(hasNeg && hasPos);
	}

	private static double sign(double px, double py, double x2, double y2, double x3, double y3) {
		return (px - x3) * (y2 - y3) - (x2 - x3) * (py - y3);
	}

	private static double distance(Pose a, Pose b) {
		double dx = a.getX() - b.getX();
		double dy = a.getY() - b.getY();
		return Math.sqrt(dx * dx + dy * dy);
	}

	/**
	 * Closest point on (or inside) the triangle to {@code p}. If {@code p} is
	 * inside the triangle, returns {@code p} clamped to the triangle via the
	 * barycentric projection.
	 */
	private static Pose nearestPointInTriangle(Pose p, double[] tri) {
		double ax = tri[0], ay = tri[1];
		double bx = tri[2], by = tri[3];
		double cx = tri[4], cy = tri[5];

		// Check if P is inside the triangle; if so, the nearest point is P itself.
		if (isInTriangle(p, tri)) {
			return p;
		}

		// Otherwise the nearest point lies on one of the three edges.
		Pose ab = closestPointOnSegment(p, ax, ay, bx, by);
		Pose bc = closestPointOnSegment(p, bx, by, cx, cy);
		Pose ca = closestPointOnSegment(p, cx, cy, ax, ay);

		double dAB = distance(p, ab);
		double dBC = distance(p, bc);
		double dCA = distance(p, ca);

		Pose best = ab;
		double bestD = dAB;
		if (dBC < bestD) { best = bc; bestD = dBC; }
		if (dCA < bestD) { best = ca; }
		return best;
	}

	private static Pose closestPointOnSegment(Pose p, double ax, double ay, double bx, double by) {
		double abx = bx - ax;
		double aby = by - ay;
		double apx = p.getX() - ax;
		double apy = p.getY() - ay;
		double abLenSq = abx * abx + aby * aby;
		double t = abLenSq == 0 ? 0 : (apx * abx + apy * aby) / abLenSq;
		if (t < 0) t = 0;
		else if (t > 1) t = 1;
		return new Pose(ax + t * abx, ay + t * aby);
	}

	/**
	 * Signed perpendicular distance from {@code p} to the nearest edge of
	 * {@code tri}. Positive = inside, negative = outside. If {@code p} is
	 * strictly inside, returns the minimum perpendicular distance to any of
	 * the three edges (always positive).
	 */
	private static double signedDistanceToTriangle(Pose p, double[] tri) {
		if (isInTriangle(p, tri)) {
			// Minimum perpendicular distance to any edge (always positive here).
			double dAB = pointToSegmentDistance(p, tri[0], tri[1], tri[2], tri[3]);
			double dBC = pointToSegmentDistance(p, tri[2], tri[3], tri[4], tri[5]);
			double dCA = pointToSegmentDistance(p, tri[4], tri[5], tri[0], tri[1]);
			return Math.min(dAB, Math.min(dBC, dCA));
		}
		// Outside: return -distance to nearest point on the triangle boundary.
		return -distance(p, nearestPointInTriangle(p, tri));
	}

	private static double pointToSegmentDistance(Pose p, double ax, double ay, double bx, double by) {
		double abx = bx - ax;
		double aby = by - ay;
		double apx = p.getX() - ax;
		double apy = p.getY() - ay;
		double abLenSq = abx * abx + aby * aby;
		double t = abLenSq == 0 ? 0 : (apx * abx + apy * aby) / abLenSq;
		if (t < 0) t = 0;
		else if (t > 1) t = 1;
		double px = ax + t * abx;
		double py = ay + t * aby;
		double dx = p.getX() - px;
		double dy = p.getY() - py;
		return Math.sqrt(dx * dx + dy * dy);
	}

	/**
	 * Heading (radians) the robot should hold so its back-mounted shooter fires
	 * toward {@code (goalX, goalY)}. Mirrors {@code Drive.calculateShotAngle}:
	 * {@code atan2(goalY - y, goalX - x) + π}.
	 */
	public static double calculateAimHeading(double x, double y, double goalX, double goalY) {
		double tanAng = Math.atan2(goalY - y, goalX - x);
		return MathFunctions.normalizeAngleSigned(tanAng + Math.PI);
	}

	/**
	 * True when the robot's current heading is within {@code toleranceRad} of
	 * the heading needed to aim the shooter at {@code (goalX, goalY)}.
	 */
	public static boolean isAimedAtGoal(Pose pose, double goalX, double goalY, double toleranceRad) {
		double desired = calculateAimHeading(pose.getX(), pose.getY(), goalX, goalY);
		double err = Math.IEEEremainder(pose.getHeading() - desired, 2 * Math.PI);
		return Math.abs(err) <= toleranceRad;
	}
}
