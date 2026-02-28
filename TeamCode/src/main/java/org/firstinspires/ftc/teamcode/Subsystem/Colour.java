package org.firstinspires.ftc.teamcode.Subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.seattlesolvers.solverslib.command.SubsystemBase;

@Configurable
public class Colour extends SubsystemBase {
	private NormalizedColorSensor slot2;
	private NormalizedColorSensor slot3;
	private int lastSpindexerSlot = -1;
	private SlotColour lastSlot2SensorColour = SlotColour.NONE;
	private SlotColour lastSlot3SensorColour = SlotColour.NONE;
	private int slot2ConfirmationCount = 0;
	private int slot3ConfirmationCount = 0;
	private static final int CONFIRMATION_THRESHOLD = 10; // ~300-500ms at 30-50ms per update cycle
	
	// Logging fields for sensor data
	public double slot2Hue = 0;
	public double slot2Saturation = 0;
	public double slot2Value = 0;
	public double slot2Red = 0;
	public double slot2Green = 0;
	public double slot2Blue = 0;
	
	public double slot3Hue = 0;
	public double slot3Saturation = 0;
	public double slot3Value = 0;
	public double slot3Red = 0;
	public double slot3Green = 0;
	public double slot3Blue = 0;
	
	public boolean isUpdating = false;
	public int updateCount = 0;
	public long lastUpdateTime = 0;

	public enum SlotColour {
		GREEN,
		PURPLE,
		NONE
	}

	public class SlotColours {
		private SlotColour slot1 = SlotColour.NONE;
		private SlotColour slot2 = SlotColour.NONE;
		private SlotColour slot3 = SlotColour.NONE;

		public SlotColour getSlot1() {
			return slot1;
		}

		public SlotColour getSlot2() {
			return slot2;
		}

		public SlotColour getSlot3() {
			return slot3;
		}

		public void setSlot1(SlotColour colour) {
			slot1 = colour;
		}

		public void setSlot2(SlotColour colour) {
			slot2 = colour;
		}

		public void setSlot3(SlotColour colour) {
			slot3 = colour;
		}
	}

	public static double[] rgbToHsv(double r, double g, double b) {
		double max = Math.max(r, Math.max(g, b));
		double min = Math.min(r, Math.min(g, b));
		double delta = max - min;
		double h = 0, s = 0, v = max;

		if (max != 0) {
			s = delta / max;
		} else {
			return new double[]{0, 0, 0}; // Black
		}

		if (delta != 0) {
			if (max == r) {
				h = (g - b) / delta + (g < b ? 6 : 0);
			} else if (max == g) {
				h = (b - r) / delta + 2;
			} else {
				h = (r - g) / delta + 4;
			}
			h /= 6;
		}

		return new double[]{h * 360, s, v};
	}

	public SlotColours colours;

	public Colour(HardwareMap hardwareMap) {
		slot2 = hardwareMap.get(NormalizedColorSensor.class, "slot2");
		slot3 = hardwareMap.get(NormalizedColorSensor.class, "slot3");
		colours = new SlotColours();
	}

	public void update(double spindexerSlot) {
		isUpdating = true;
		updateCount++;
		lastUpdateTime = System.currentTimeMillis();
		
		int slot = (int) spindexerSlot;
		boolean spindexerTurned = (slot != lastSpindexerSlot);
		lastSpindexerSlot = slot;

		// Read colors from sensors
		SlotColour slot2SensorColour = detectColour(slot2, true);
		SlotColour slot3SensorColour = detectColour(slot3, false);

		// Apply confirmation filter when spindexer hasn't turned
		// This prevents noise from overwriting colors while still allowing corrections
		SlotColour filteredSlot2Colour = filterColour(slot2SensorColour, lastSlot2SensorColour, slot2ConfirmationCount, spindexerTurned);
		SlotColour filteredSlot3Colour = filterColour(slot3SensorColour, lastSlot3SensorColour, slot3ConfirmationCount, spindexerTurned);

		// Update confirmation counters
		if (slot2SensorColour == lastSlot2SensorColour) {
			slot2ConfirmationCount = 0;
		} else {
			slot2ConfirmationCount++;
		}
		if (slot3SensorColour == lastSlot3SensorColour) {
			slot3ConfirmationCount = 0;
		} else {
			slot3ConfirmationCount++;
		}

		lastSlot2SensorColour = slot2SensorColour;
		lastSlot3SensorColour = slot3SensorColour;

		// Map sensors to slots based on spindexer position
		// When spindexerSlot is 1: slot2 sensor reads slot2, slot3 sensor reads slot3
		// When spindexerSlot is 2: slot2 sensor reads slot3, slot3 sensor reads slot1
		// When spindexerSlot is 3: slot2 sensor reads slot1, slot3 sensor reads slot2
		switch (slot) {
			case 1:
				colours.setSlot2(filteredSlot2Colour);
				colours.setSlot3(filteredSlot3Colour);
				break;
			case 2:
				colours.setSlot3(filteredSlot2Colour);
				colours.setSlot1(filteredSlot3Colour);
				break;
			case 3:
				colours.setSlot1(filteredSlot2Colour);
				colours.setSlot2(filteredSlot3Colour);
				break;
		}
		
		isUpdating = false;
	}

	private SlotColour filterColour(SlotColour current, SlotColour last, int confirmationCount, boolean spindexerTurned) {
		// If spindexer turned, trust the new reading immediately
		if (spindexerTurned) {
			return current;
		}
		// If the reading has been consistent for enough cycles, accept the change
		if (confirmationCount >= CONFIRMATION_THRESHOLD) {
			return current;
		}
		// Otherwise, keep the last known color (be hesitant to change)
		return last;
	}

	/**
	 * Sets the slot at the "slot1 position" (when spindexerSlot is 1) to NONE.
	 * When spindexerSlot is 1, sets slot1 to NONE.
	 * When spindexerSlot is 2, sets slot2 to NONE.
	 * When spindexerSlot is 3, sets slot3 to NONE.
	 * @param spindexerSlot the current spindexer slot position
	 */
	public void clearCurrentSlot(double spindexerSlot) {
		int slot = (int) spindexerSlot;
		switch (slot) {
			case 1:
				colours.setSlot1(SlotColour.NONE);
				break;
			case 2:
				colours.setSlot2(SlotColour.NONE);
				break;
			case 3:
				colours.setSlot3(SlotColour.NONE);
				break;
		}
	}

	private SlotColour detectColour(NormalizedColorSensor sensor, boolean isSlot2) {
		// Get normalized color values from sensor (read once for consistency)
		NormalizedRGBA colors = sensor.getNormalizedColors();
		double r = colors.red;
		double g = colors.green;
		double b = colors.blue;

		// Convert to HSV for easier color detection
		double[] hsv = rgbToHsv(r, g, b);
		double hue = hsv[0];
		double saturation = hsv[1];
		double value = hsv[2];

		// Store values for logging
		if (isSlot2) {
			slot2Red = r;
			slot2Green = g;
			slot2Blue = b;
			slot2Hue = hue;
			slot2Saturation = saturation;
			slot2Value = value;
		} else {
			slot3Red = r;
			slot3Green = g;
			slot3Blue = b;
			slot3Hue = hue;
			slot3Saturation = saturation;
			slot3Value = value;
		}

		// Check for green (hue around 80-160 degrees)
		if (saturation > 0.3 && hue >= 80 && hue <= 160) {
			return SlotColour.GREEN;
		}

		// Check for purple (hue around 270-330 degrees)
		if (saturation > 0.3 && hue >= 270 && hue <= 330) {
			return SlotColour.PURPLE;
		}

		return SlotColour.NONE;
	}
}
