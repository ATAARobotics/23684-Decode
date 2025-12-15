package org.firstinspires.ftc.teamcode.Utilities.Animations;

import android.graphics.Color;

/**
 * Animation that pulses the intensity (brightness) of a color.
 */
public class PulseAnimation implements Animation {
    private final String baseColor;
    private final float minValue;
    private final float maxValue;
    private final long periodMs;
    private final float[] baseHsv = new float[3];
    private final float[] currentHsv = new float[3];
    private long elapsedTimeMs = 0;

    /**
     * @param baseColor Base color to pulse (hex string).
     * @param minValue Minimum brightness (0.0 to 1.0).
     * @param maxValue Maximum brightness (0.0 to 1.0).
     * @param periodMs Period of one pulse cycle in ms.
     */
    public PulseAnimation(String baseColor, float minValue, float maxValue, long periodMs) {
        this.baseColor = baseColor;
        this.minValue = minValue;
        this.maxValue = maxValue;
        this.periodMs = periodMs;

        int colorInt = (int) Long.parseLong(baseColor.startsWith("#") ? baseColor.substring(1) : baseColor, 16);
        Color.colorToHSV(colorInt, baseHsv);
    }

    @Override
    public void update(long deltaTimeMs) {
        elapsedTimeMs += deltaTimeMs;
    }

    @Override
    public String getColor() {
        // Sine wave for smooth pulsing
        double t = (elapsedTimeMs % periodMs) / (double) periodMs;
        double sine = Math.sin(2 * Math.PI * t);
        float value = minValue + (maxValue - minValue) * (float) ((sine + 1) / 2);

        currentHsv[0] = baseHsv[0];
        currentHsv[1] = baseHsv[1];
        currentHsv[2] = value;

        int color = Color.HSVToColor(currentHsv);
        return String.format("#%06X", color & 0xFFFFFF);
    }

    @Override
    public boolean isFinished() {
        return false; // Infinite pulse
    }

    @Override
    public void reset() {
        elapsedTimeMs = 0;
    }
}