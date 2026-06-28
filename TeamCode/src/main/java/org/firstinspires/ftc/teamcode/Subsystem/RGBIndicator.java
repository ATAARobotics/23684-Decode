package org.firstinspires.ftc.teamcode.Subsystem;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

/**
 * RGB indicator subsystem that drives the GoBilda RGB Indicator Light based on
 * beam-breaker ball count and shooter state / field position.
 * <p>
 * Color decision tree (top-down, first match wins):
 * <ol>
 *   <li>Shooter triggered:
 *     <ul>
 *       <li>Outside both shooting zones: rapid flash between shooter-status (green/red) and white.</li>
 *       <li>Inside a shooting zone: solid green if shooter at target, solid red otherwise.</li>
 *     </ul>
 *   </li>
 *   <li>Shooter not triggered: ball-count driven color (purple/yellow/azure, or slow blue/violet flash at 3+).</li>
 * </ol>
 * The at-target check uses {@link Transfer#reachedAverageTarget} (tolerance 150 RPM)
 * to match the pre-refactor behavior of {@code MainTeleOp.updateRGBIndicator()}.
 * <p>
 * {@link #periodic()} is invoked by the {@code CommandScheduler} each loop — do not
 * call it explicitly or the flash timing will be evaluated twice per loop.
 */
public class RGBIndicator extends SubsystemBase {

	public static final double SERVO_IDLE_PURPLE = 0.722;
	public static final double SERVO_YELLOW = 0.388;
	public static final double SERVO_AZURE = 0.555;
	public static final double SERVO_GREEN = 0.500;
	public static final double SERVO_RED = 0.28;
	public static final double SERVO_BLUE = 0.611;
	public static final double SERVO_VIOLET = 0.722;
	public static final double SERVO_WHITE = 1.0;

	public static final long SLOW_FLASH_PERIOD_MS = 1000;
	public static final long RAPID_FLASH_PERIOD_MS = 250;

	private static final double[] ZONE_A = {0.0, 144.0, 72.0, 72.0, 144.0, 144.0};
	private static final double[] ZONE_B = {48.0, 0.0, 72.0, 24.0, 96.0, 0.0};

	private final Servo rgbServo;
	private Transfer transfer;
	private BeamBreaker beamBreaker;
	private Follower follower;
	private boolean shooterTriggered = false;

	public RGBIndicator(HardwareMap hardwareMap) {
		rgbServo = hardwareMap.get(Servo.class, "rgbIndicator");
	}

	public void setTransfer(Transfer transfer) {
		this.transfer = transfer;
	}

	public void setBeamBreaker(BeamBreaker beamBreaker) {
		this.beamBreaker = beamBreaker;
	}

	public void setFollower(Follower follower) {
		this.follower = follower;
	}

	public void setShooterTriggered(boolean triggered) {
		this.shooterTriggered = triggered;
	}

	@Override
	public void periodic() {
		long now = System.currentTimeMillis();
		double position;

		if (shooterTriggered && transfer != null && follower != null) {
			boolean atTarget = transfer.reachedAverageTarget;
			Pose pose = follower.getPose();
			boolean inZone = isInTriangle(pose, ZONE_A) || isInTriangle(pose, ZONE_B);

			if (!inZone) {
				boolean flashOn = (now % (2 * RAPID_FLASH_PERIOD_MS)) < RAPID_FLASH_PERIOD_MS;
				position = flashOn ? (atTarget ? SERVO_GREEN : SERVO_RED) : SERVO_WHITE;
			} else {
				position = atTarget ? SERVO_GREEN : SERVO_RED;
			}
		} else if (beamBreaker != null) {
			int count = beamBreaker.getBallCount();
			if (count <= 0) {
				position = SERVO_IDLE_PURPLE;
			} else if (count == 1) {
				position = SERVO_YELLOW;
			} else if (count == 2) {
				position = SERVO_AZURE;
			} else {
				boolean flashOn = (now % (2 * SLOW_FLASH_PERIOD_MS)) < SLOW_FLASH_PERIOD_MS;
				position = flashOn ? SERVO_BLUE : SERVO_VIOLET;
			}
		} else {
			position = SERVO_IDLE_PURPLE;
		}

		rgbServo.setPosition(position);
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
}
