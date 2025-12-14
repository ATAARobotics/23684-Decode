package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Singleton subsystem that manages dual touch sensors for contact detection.
 * <p>
 * Reads raw touch data from both sensors and provides a combined detection state.
 * Useful for limit switches, collision detection, and mechanical position sensing.
 * <p>
 * To use:
 * 1. Call initialize(hardwareMap) once during robot initialization
 * 2. Call getInstance().update().run(packet) in the main loop
 * 3. Read touchLeft and touchRight for individual sensor states
 * 4. Read detected for combined state (true if either sensor is pressed)
 */
public class TouchDetector {
	private static TouchDetector instance = null;
	// Public fields to store the last read values
	/**
	 * True if left touch sensor is pressed
	 */
	public boolean touchLeft;
	/**
	 * True if right touch sensor is pressed
	 */
	public boolean touchRight;
	/**
	 * True if either touch sensor is pressed (OR logic)
	 */
	public boolean detected;
	private TouchSensor sensorLeft;
	private TouchSensor sensorRight;

	private TouchDetector() {
	}

	public static void initialize(HardwareMap hardwareMap) {
		instance = new TouchDetector();
		instance.sensorLeft = hardwareMap.get(TouchSensor.class, "touchLeft");
		instance.sensorRight = hardwareMap.get(TouchSensor.class, "touchRight");
	}

	public static TouchDetector getInstance() {
		if (instance == null) {
			throw new IllegalStateException("TouchDetector not initialized. Call initialize(hardwareMap) first.");
		}
		return instance;
	}

	public static void shutdown() {
		// No cleanup needed currently
	}

	/**
	 * Reads values from both touch sensors and updates detection state.
	 * Updates the public touchLeft, touchRight, and detected fields.
	 */
	private void updateValues() {
		touchLeft = sensorLeft.isPressed();
		touchRight = sensorRight.isPressed();
		detected = touchLeft || touchRight; // True if either sensor is pressed
	}

	/**
	 * Returns an Action that updates touch sensor readings.
	 * This action finishes immediately after one frame and should be called every loop.
	 *
	 * @return Action that performs one sensor update
	 */
	public Action update() {
		return new UpdateAction();
	}

	/**
	 * Internal Action that performs touch sensor updates.
	 * Updates sensor readings and logs telemetry data.
	 */
	private class UpdateAction implements Action {
		@Override
		public boolean run(@NonNull TelemetryPacket packet) {
			updateValues();
			packet.put("Touch Left", touchLeft);
			packet.put("Touch Right", touchRight);
			packet.put("Detected", detected);
			return false; // Action finishes immediately after one frame
		}
	}
}
