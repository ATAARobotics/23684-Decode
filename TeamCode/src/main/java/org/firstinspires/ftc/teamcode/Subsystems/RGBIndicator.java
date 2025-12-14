package org.firstinspires.ftc.teamcode.Subsystems;

import android.graphics.Color;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class RGBIndicator {

	// --- Tuning Parameters ---
	// If saturation is below this (0.0 to 1.0), the light turns White.
	public static double SATURATION_THRESHOLD = 0.2;

	// goBILDA 3118-0808-0002 Servo Values
	private static final double VAL_RED = 0.277;
	private static final double VAL_ORANGE = 0.333;
	private static final double VAL_YELLOW = 0.388;
	private static final double VAL_SAGE = 0.444;
	private static final double VAL_GREEN = 0.500;
	private static final double VAL_AZURE = 0.555;
	private static final double VAL_BLUE = 0.611;
	private static final double VAL_VIOLET = 0.722;
	private static final double VAL_WHITE = 1.0;
	private static final double VAL_OFF = 0.0;

	private static RGBIndicator instance = null;
	private Servo rgbServo;

	// Reusable array to prevent garbage collection churn
	private final float[] hsvCache = new float[3];

	private RGBIndicator() {}

	public static void initialize(HardwareMap hardwareMap) {
		instance = new RGBIndicator();
		if (hardwareMap.servo.contains("rgbIndicator")) {
			instance.rgbServo = hardwareMap.get(Servo.class, "rgbIndicator");
		}
	}

	public static RGBIndicator getInstance() {
		if (instance == null) instance = new RGBIndicator();
		return instance;
	}

	public static void shutdown() {
		// No cleanup needed currently
	}

	/**
	 * Optimized setColor using HSV.
	 * Execution time: ~0.02ms
	 */
	public void setColor(String hexColor) {
		if (rgbServo == null) return;

		// 1. Fast Hex Parsing
		if (hexColor.startsWith("#")) hexColor = hexColor.substring(1);
		int color;
		try {
			// optimized parsing
			color = (int) Long.parseLong(hexColor, 16);
		} catch (NumberFormatException e) {
			return;
		}

		// 2. Convert to HSV (Native Android Method - Extremely Fast)
		// hsv[0] = Hue (0..360)
		// hsv[1] = Saturation (0..1)
		// hsv[2] = Value (0..1)
		Color.colorToHSV(color, hsvCache);
		float hue = hsvCache[0];
		float sat = hsvCache[1];
		float val = hsvCache[2];

		double servoPos;

		// 3. Logic Mapping
		if (val < 0.1) {
			servoPos = VAL_OFF; // Black/Off
		} else if (sat < SATURATION_THRESHOLD) {
			servoPos = VAL_WHITE; // Low saturation = White
		} else {
			servoPos = mapHueToServo(hue);
		}

		rgbServo.setPosition(servoPos);
	}

	/**
	 * Maps 0-360 Hue to 0.277-0.722 Servo Position.
	 * Uses linear interpolation between defined goBILDA points.
	 */
	private double mapHueToServo(float hue) {
		// Hue 0 is Red, Hue 360 is also Red.
		// The goBILDA scale is linear with the spectrum (Red -> Violet).
		// However, Hue wraps around (Violet -> Red is 270 -> 360/0).

		// --- Segment 1: Red (0) to Green (120) ---
		if (hue <= 30) {
			// Red (0) to Orange (30)
			return map(hue, 0, 30, VAL_RED, VAL_ORANGE);
		} else if (hue <= 60) {
			// Orange (30) to Yellow (60)
			return map(hue, 30, 60, VAL_ORANGE, VAL_YELLOW);
		} else if (hue <= 90) {
			// Yellow (60) to Sage/Lime (90)
			return map(hue, 60, 90, VAL_YELLOW, VAL_SAGE);
		} else if (hue <= 120) {
			// Sage (90) to Green (120)
			return map(hue, 90, 120, VAL_SAGE, VAL_GREEN);
		}

		// --- Segment 2: Green (120) to Blue (240) ---
		else if (hue <= 180) {
			// Green (120) to Azure/Cyan (180)
			return map(hue, 120, 180, VAL_GREEN, VAL_AZURE);
		} else if (hue <= 240) {
			// Azure (180) to Blue (240)
			return map(hue, 180, 240, VAL_AZURE, VAL_BLUE);
		}

		// --- Segment 3: Blue (240) to Violet (270+) ---
		else if (hue <= 275) {
			// Blue (240) to Indigo/Violet (275)
			return map(hue, 240, 275, VAL_BLUE, VAL_VIOLET);
		}

		// --- Segment 4: Purples/Magentas (275 to 360) ---
		// These don't exist in the rainbow. They are "wrapped" around.
		// We map them to the Violet end of the servo range.
		else {
			return VAL_VIOLET;
		}
	}

	// Simple linear map function
	private double map(double x, double in_min, double in_max, double out_min, double out_max) {
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}

	// --- Standard Actions ---
	public void setDirectPosition(double position) {
		if (rgbServo != null) rgbServo.setPosition(position);
	}

	public Action setDirectPositionAction(double position) {
		return new InstantAction(() -> setDirectPosition(position));
	}

	public Action setColorAction(String hexColor) {
		return new InstantAction(() -> setColor(hexColor));
	}
}
