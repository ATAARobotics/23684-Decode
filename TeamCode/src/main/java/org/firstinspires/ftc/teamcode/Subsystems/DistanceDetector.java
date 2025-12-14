package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Singleton subsystem that manages a distance sensor for object detection.
 * <p>
 * Reads distance measurements and triggers isObject when an object is detected
 * within 2 cm.
 * Configuration values can be tuned via FTC Dashboard.
 * <p>
 * To use:
 * 1. Call initialize(hardwareMap) once during robot initialization
 * 2. Call getInstance().update().run(packet) in the main loop
 * 3. Read rawDistance for the current distance in centimeters
 * 4. Read isObject boolean field for object detection result
 */
@Config
public class DistanceDetector {

	// Detection threshold (tunable via FTC Dashboard)
	/**
	 * Distance threshold in centimeters for object detection
	 */
	public static double OBJECT_THRESHOLD_CM = 2.0;

	private static DistanceDetector instance = null;
	// Public fields to store the last read values
	/**
	 * Raw distance reading in centimeters
	 */
	public double rawDistance;
	/**
	 * True if an object is detected within OBJECT_THRESHOLD_CM
	 */
	public boolean isObject;
	private DistanceSensor intakeDistanceSensor;

	private DistanceDetector() {
	}

	public static void initialize(HardwareMap hardwareMap) {
		instance = new DistanceDetector();
		instance.intakeDistanceSensor = hardwareMap.get(DistanceSensor.class, "intakeDistanceSensor");
	}

	public static DistanceDetector getInstance() {
		if (instance == null) {
			throw new IllegalStateException("DistanceDetector not initialized. Call initialize(hardwareMap) first.");
		}
		return instance;
	}

	public static void shutdown() {
		// No cleanup needed currently
	}

	/**
	 * Reads value from distance sensor and updates detection state.
	 * Updates the public rawDistance field and checks against the threshold.
	 * Updates the public isObject field.
	 */
	private void updateValues() {
		rawDistance = intakeDistanceSensor.getDistance(DistanceUnit.CM);
		isObject = rawDistance <= OBJECT_THRESHOLD_CM;
	}

	/**
	 * Returns an Action that updates distance sensor readings.
	 * This action finishes immediately after one frame and should be called every loop.
	 *
	 * @return Action that performs one sensor update
	 */
	public Action update() {
		return new UpdateAction();
	}

	/**
	 * Internal Action that performs distance sensor updates.
	 * Updates sensor readings and logs telemetry data.
	 */
	private class UpdateAction implements Action {
		@Override
		public boolean run(@NonNull TelemetryPacket packet) {
			updateValues();
			packet.put("Distance (cm)", rawDistance);
			packet.put("Is Object", isObject);
			return false; // Action finishes immediately after one frame
		}
	}
}
