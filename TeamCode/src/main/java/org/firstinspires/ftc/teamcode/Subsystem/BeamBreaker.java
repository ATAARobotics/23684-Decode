package org.firstinspires.ftc.teamcode.Subsystem;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

/**
 * Beam breaker subsystem for counting artifacts entering/exiting the robot.
 * <p>
 * Tracks ball count with edge detection that is aware of intake direction.
 */
public class BeamBreaker extends SubsystemBase {

	private final DigitalChannel intakeBeamBreaker;

	/**
	 * Current count of balls in the robot.
	 */
	private int ballCount = 0;

	/**
	 * Previous beam state for edge detection
	 */
	private boolean lastBeamState = false;

	/**
	 * Whether intake was last detected running IN
	 */
	private boolean lastIntakeIn = true;

	/**
	 * Timestamp of last valid edge (milliseconds)
	 */
	private long lastEdgeTime = 0;

	/**
	 * Minimum time between valid edges to suppress noise
	 */
	private static final long DEBOUNCE_MS = 50;

	/**
	 * Noise suppression window for rapid break→clear→break sequences (ms)
	 */
	private static final long NOISE_SUPPRESS_MS = 300;

	/**
	 * Time of last broken→clear transition for noise detection
	 */
	private long lastClearTime = 0;

	/**
	 * Last event description for telemetry
	 */
	private String lastEvent = "None";

	/**
	 * Last direction logged ("IN" or "OUT")
	 */
	private String lastDirection = "STOPPED";

	/**
	 * Whether an edge was debounced this loop
	 */
	private boolean debounceActive = false;

	public BeamBreaker(HardwareMap hardwareMap) {
		intakeBeamBreaker = hardwareMap.get(DigitalChannel.class, "intakeBeamBreaker");
		intakeBeamBreaker.setMode(DigitalChannel.Mode.INPUT);
	}

	/**
	 * Check if the beam is currently broken.
	 *
	 * @return true when beam is blocked (object present), false otherwise
	 */
	public boolean isBeamBroken() {
		return intakeBeamBreaker.getState();
	}

	/**
	 * Command that waits until the beam is cut.
	 *
	 * @return command that completes when beam breaks
	 */
	public Command WaitForBeamCut() {
		return new WaitUntilCommand(() -> isBeamBroken());
	}

	/**
	 * Get the current ball count.
	 *
	 * @return number of balls currently tracked in the robot
	 */
	public int getBallCount() {
		return ballCount;
	}

	/**
	 * Set the ball count directly (for override).
	 *
	 * @param count new ball count value
	 */
	public void setBallCount(int count) {
		ballCount = count;
	}

	/**
	 * Reset the ball count to zero.
	 */
	public void resetBallCount() {
		ballCount = 0;
		lastEvent = "RESET";
	}

	/**
	 * Update beam breaker state each loop.
	 * <p>
	 * Performs edge detection with direction awareness:
	 * <ul>
	 *   <li>Broken → Clear (ball exits): decrements count if intake was running OUT</li>
	 *   <li>Clear → Broken (ball enters): increments count if intake was running IN</li>
	 * </ul>
	 * Debounces rapid transitions and suppresses noise from balls passing
	 * through without fully clearing.
	 *
	 * @param intakeRunningIn true if intake is currently running IN, false if OUT, null if stopped
	 */
	public void update(Boolean intakeRunningIn) {
		boolean currentBeamState = isBeamBroken();
		debounceActive = false;

		if (intakeRunningIn != null) {
			lastIntakeIn = intakeRunningIn;
		}

		final long now = System.currentTimeMillis();
		final boolean beamChanged = (currentBeamState != lastBeamState);

		if (beamChanged) {
			if ((now - lastEdgeTime) < DEBOUNCE_MS) {
				debounceActive = true;
				lastEdgeTime = now;
				lastBeamState = currentBeamState;
				return;
			}

			// When intake is stopped, don't count edges but still track state
			if (intakeRunningIn == null) {
				lastEdgeTime = now;
				lastBeamState = currentBeamState;
				return;
			}

			if (!currentBeamState && lastBeamState) {
				lastClearTime = now;
			}

			if (currentBeamState && !lastBeamState && !debounceActive) {
				if (!lastClearTimeIsNoise(now)) {
					if (lastIntakeIn) {
						ballCount++;
						lastEvent = "CLEAR→BROKEN";
						lastDirection = "IN";
					}
				} else {
					lastEvent = "NOISE FILTERED";
					lastDirection = lastIntakeIn ? "IN" : "OUT";
				}
			} else if (!currentBeamState && lastBeamState && !debounceActive) {
				if (!lastIntakeIn) {
					ballCount = Math.max(0, ballCount - 1);
					lastEvent = "BROKEN→CLEAR";
					lastDirection = "OUT";
				}
			}

			lastEdgeTime = now;
			lastBeamState = currentBeamState;
		}
	}

	private boolean lastClearTimeIsNoise(long now) {
		return lastClearTime > 0 && (now - lastClearTime) < NOISE_SUPPRESS_MS;
	}

	/**
	 * Output beam breaker telemetry to Panels.
	 *
	 * @param telemetry Panels telemetry wrapper
	 */
	public void telemetry(TelemetryManager.TelemetryWrapper telemetry) {
		String intakeState = lastIntakeIn ? "IN" : "OUT";
		telemetry.addLine("=== BEAM BREAKER ===");
		telemetry.addData("Beam Broken", isBeamBroken());
		telemetry.addData("Ball Count", ballCount);
		telemetry.addData("Last Event", lastEvent);
		telemetry.addData("Last Direction", lastDirection);
		telemetry.addData("Debounce Active", debounceActive);
		telemetry.addData("Intake Running", intakeState);
	}
}
