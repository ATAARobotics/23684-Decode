package org.firstinspires.ftc.teamcode.Utilities.Animations;

import android.graphics.Color;

/**
 * Animation that smoothly fades a color in and out (breathing effect).
 */
public class BreathingAnimation implements Animation {
    private final String baseColor;
    private final long periodMs;
    private final float[] baseHsv = new float[3];
    private final float[] currentHsv = new float[3];
    private long elapsedTimeMs = 0;

    /**
     * @param baseColor Base color to breathe (hex string).
     * @param periodMs Period of one breathe cycle in ms.
     */
    public BreathingAnimation(String baseColor, long periodMs) {
        this.baseColor = baseColor;
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
        // Sine wave for smooth breathing: value from 0.1 to 1.0
        double t = (elapsedTimeMs % periodMs) / (double) periodMs;
        double sine = Math.sin(2 * Math.PI * t);
        float value = 0.1f + 0.9f * (float) ((sine + 1) / 2);

        currentHsv[0] = baseHsv[0];
        currentHsv[1] = baseHsv[1];
        currentHsv[2] = value;

        int color = Color.HSVToColor(currentHsv);
        return String.format("#%06X", color & 0xFFFFFF);
    }

    @Override
    public boolean isFinished() {
        return false; // Infinite breathing
    }

    @Override
    public void reset() {
        elapsedTimeMs = 0;
    }
}