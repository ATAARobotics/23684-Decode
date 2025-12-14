package org.firstinspires.ftc.teamcode.Subsystems;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Singleton subsystem that manages dual color sensors for ball color detection.
 * <p>
 * Uses HSV color space to detect green and purple balls.
 * Configuration values can be tuned via FTC Dashboard.
 * <p>
 * To use:
 * 1. Call initialize(hardwareMap) once during robot initialization
 * 2. Call getInstance().update().run(packet) in the main loop
 * 3. Read isGreen and isPurple boolean fields for detection results
 */
@Config
public class ColorDetector {

	// HSV Thresholds (tunable via FTC Dashboard)
	/**
	 * Minimum hue value for green detection
	 */
	public static double GREEN_HUE_MIN = 100;
	/**
	 * Maximum hue value for green detection
	 */
	public static double GREEN_HUE_MAX = 140;
	/**
	 * Minimum hue value for purple detection
	 */
	public static double PURPLE_HUE_MIN = 250;
	/**
	 * Maximum hue value for purple detection
	 */
	public static double PURPLE_HUE_MAX = 290;
	/**
	 * Minimum saturation required to consider a color valid
	 */
	public static double MIN_SATURATION = 0.4;

	private static ColorDetector instance = null;
	
	// Left sensor values
	/**
	 * Red value from left sensor (0-255)
	 */
	public int leftRed;
	/**
	 * Green value from left sensor (0-255)
	 */
	public int leftGreen;
	/**
	 * Blue value from left sensor (0-255)
	 */
	public int leftBlue;
	/**
	 * HSV values for left sensor: [0]=hue (0-360), [1]=saturation (0-1), [2]=value (0-1)
	 */
	public float[] leftHSV = new float[3];
	/**
	 * True if the left sensor detected a green ball
	 */
	public boolean leftIsGreen;
	/**
	 * True if the left sensor detected a purple ball
	 */
	public boolean leftIsPurple;
	
	// Right sensor values
	/**
	 * Red value from right sensor (0-255)
	 */
	public int rightRed;
	/**
	 * Green value from right sensor (0-255)
	 */
	public int rightGreen;
	/**
	 * Blue value from right sensor (0-255)
	 */
	public int rightBlue;
	/**
	 * HSV values for right sensor: [0]=hue (0-360), [1]=saturation (0-1), [2]=value (0-1)
	 */
	public float[] rightHSV = new float[3];
	/**
	 * True if the right sensor detected a green ball
	 */
	public boolean rightIsGreen;
	/**
	 * True if the right sensor detected a purple ball
	 */
	public boolean rightIsPurple;
	
	private com.qualcomm.robotcore.hardware.ColorSensor colourLeft;
	private com.qualcomm.robotcore.hardware.ColorSensor colourRight;

	private ColorDetector() {
	}

	public static void initialize(HardwareMap hardwareMap) {
		instance = new ColorDetector();
		instance.colourLeft = hardwareMap.get(com.qualcomm.robotcore.hardware.ColorSensor.class, "colourLeft");
		instance.colourRight = hardwareMap.get(com.qualcomm.robotcore.hardware.ColorSensor.class, "colourRight");
	}

	public static ColorDetector getInstance() {
		if (instance == null) {
			throw new IllegalStateException("ColorSensor not initialized. Call initialize(hardwareMap) first.");
		}
		return instance;
	}

	public static void shutdown() {
		// No cleanup needed currently
	}

	/**
	 * Reads values from both color sensors and updates detection state.
	 * Performs individual calculations for each sensor.
	 * Converts to HSV and checks against thresholds.
	 * Updates all public fields for individual sensors and their detection state.
	 */
	private void updateValues() {
		// Read left sensor
		leftRed = colourLeft.red();
		leftGreen = colourLeft.green();
		leftBlue = colourLeft.blue();
		Color.RGBToHSV(leftRed, leftGreen, leftBlue, leftHSV);
		leftIsGreen = (leftHSV[1] > MIN_SATURATION && leftHSV[0] > GREEN_HUE_MIN && leftHSV[0] < GREEN_HUE_MAX);
		leftIsPurple = (leftHSV[1] > MIN_SATURATION && leftHSV[0] > PURPLE_HUE_MIN && leftHSV[0] < PURPLE_HUE_MAX);

		// Read right sensor
		rightRed = colourRight.red();
		rightGreen = colourRight.green();
		rightBlue = colourRight.blue();
		Color.RGBToHSV(rightRed, rightGreen, rightBlue, rightHSV);
		rightIsGreen = (rightHSV[1] > MIN_SATURATION && rightHSV[0] > GREEN_HUE_MIN && rightHSV[0] < GREEN_HUE_MAX);
		rightIsPurple = (rightHSV[1] > MIN_SATURATION && rightHSV[0] > PURPLE_HUE_MIN && rightHSV[0] < PURPLE_HUE_MAX);
	}

	/**
	 * Returns an Action that updates color sensor readings.
	 * This action finishes immediately after one frame and should be called every loop.
	 *
	 * @return Action that performs one sensor update
	 */
	public Action update() {
		return new UpdateAction();
	}

	/**
	 * Internal Action that performs color sensor updates.
	 * Updates sensor readings and logs telemetry data for each sensor separately.
	 */
	private class UpdateAction implements Action {
		@Override
		public boolean run(@NonNull TelemetryPacket packet) {
			updateValues();
			
			// Left sensor telemetry
			packet.put("Left Red", leftRed);
			packet.put("Left Green", leftGreen);
			packet.put("Left Blue", leftBlue);
			packet.put("Left Hue", leftHSV[0]);
			packet.put("Left Saturation", leftHSV[1]);
			packet.put("Left Value", leftHSV[2]);
			packet.put("Left Is Green", leftIsGreen);
			packet.put("Left Is Purple", leftIsPurple);
			
			// Right sensor telemetry
			packet.put("Right Red", rightRed);
			packet.put("Right Green", rightGreen);
			packet.put("Right Blue", rightBlue);
			packet.put("Right Hue", rightHSV[0]);
			packet.put("Right Saturation", rightHSV[1]);
			packet.put("Right Value", rightHSV[2]);
			packet.put("Right Is Green", rightIsGreen);
			packet.put("Right Is Purple", rightIsPurple);
			
			return false; // Action finishes immediately after one frame
		}
	}
}
