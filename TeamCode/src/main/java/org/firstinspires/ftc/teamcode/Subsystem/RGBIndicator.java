package org.firstinspires.ftc.teamcode.Subsystem;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Utils.ShootingZone;

/**
 * RGB indicator subsystem that drives the GoBilda RGB Indicator Light based on
 * beam-breaker ball count and shooter state / field position.
 * <p>
 * Color decision tree (top-down, first match wins):
 * <ol>
 *   <li>Shooter triggered:
 *     <ul>
 *       <li>None of the 13.5&times;15.5 robot footprint corners lie in either shooting zone triangle: rapid flash between shooter-status (green/red) and white.</li>
 *       <li>At least one corner of the robot footprint lies in a shooting zone: solid green if shooter at target, solid red otherwise.</li>
 *     </ul>
 *   </li>
 *   <li>Shooter not triggered: ball-count driven color (purple/yellow/azure, or slow blue/violet flash at 3+).</li>
 * </ol>
 * The at-target check uses {@link Transfer#reachedAverageTarget} (tolerance matches
 * {@link Transfer#SHOOTER_RPM_TOLERANCE}) to match the pre-refactor behavior of
 * {@code MainTeleOp.updateRGBIndicator()}.
 * <p>
 * {@link #periodic()} is invoked by the {@code CommandScheduler} each loop — do not
 * call it explicitly or the flash timing will be evaluated twice per loop.
 */
public class RGBIndicator extends SubsystemBase {

	public static final double SERVO_IDLE_PURPLE = 0.722;
	public static final double SERVO_YELLOW = 0.388;
	public static final double SERVO_AZURE = 0.555;
	public static final double SERVO_GREEN = 0.500;
	public static final double SERVO_RED = 0.277;
	public static final double SERVO_BLUE = 0.611;
	public static final double SERVO_VIOLET = 0.722;
	public static final double SERVO_WHITE = 1.0;

	public static final long SLOW_FLASH_PERIOD_MS = 1000;
	public static final long RAPID_FLASH_PERIOD_MS = 250;

	private final Servo rgbServo;
	private Transfer transfer;
	private BeamBreaker beamBreaker;
	private Follower follower;
	private org.firstinspires.ftc.teamcode.Utils.Team team = org.firstinspires.ftc.teamcode.Utils.Team.BLUE;
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

	public void setTeam(org.firstinspires.ftc.teamcode.Utils.Team team) {
		if (team == org.firstinspires.ftc.teamcode.Utils.Team.UNKNOWN) {
			throw new IllegalArgumentException("team must be RED or BLUE; was UNKNOWN");
		}
		this.team = team;
	}

	public void setShooterTriggered(boolean triggered) {
		this.shooterTriggered = triggered;
	}

	@Override
	public void periodic() {
		long now = System.currentTimeMillis();
		double position;

		if (shooterTriggered && transfer != null && follower != null) {
			Pose pose = follower.getPose();
			boolean inZone = (team == org.firstinspires.ftc.teamcode.Utils.Team.RED)
					? ShootingZone.isAnyCornerInRedZone(pose)
					: ShootingZone.isAnyCornerInBlueZone(pose);

			if (!inZone) {
				boolean flashOn = (now % (2 * RAPID_FLASH_PERIOD_MS)) < RAPID_FLASH_PERIOD_MS;
				position = flashOn ? SERVO_RED : SERVO_WHITE;
			} else if (transfer.reachedAverageTarget) {
				position = SERVO_GREEN;
			} else {
				position = SERVO_RED;
			}
		} else if (beamBreaker != null) {
			int count = beamBreaker.getBallCount();
			if (count <= 0) {
				position = SERVO_IDLE_PURPLE;
			} else if (count == 1) {
				position = SERVO_YELLOW;
			} else if (count == 2) {
				position = SERVO_AZURE;
			} else if (count == 3) {
				boolean flashOn = (now % (2 * SLOW_FLASH_PERIOD_MS)) < SLOW_FLASH_PERIOD_MS;
				position = flashOn ? SERVO_BLUE : SERVO_GREEN;
			} else {
				boolean flashOn = (now % (2 * RAPID_FLASH_PERIOD_MS)) < RAPID_FLASH_PERIOD_MS;
				position = flashOn ? SERVO_RED : SERVO_BLUE;
			}
		} else {
			position = SERVO_IDLE_PURPLE;
		}

		rgbServo.setPosition(position);
	}
}
