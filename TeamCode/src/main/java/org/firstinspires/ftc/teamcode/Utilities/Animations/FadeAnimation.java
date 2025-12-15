package org.firstinspires.ftc.teamcode.Utilities.Animations;

import android.graphics.Color;

/**
 * Animation that fades from one color to another over a duration.
 */
public class FadeAnimation implements Animation {
    private final String startColor;
    private final String endColor;
    private final long durationMs;
    private final float[] startHsv = new float[3];
    private final float[] endHsv = new float[3];
    private final float[] currentHsv = new float[3];

    /**
     * @param startColor Starting color (hex string).
     * @param endColor Ending color (hex string).
     * @param durationMs Total fade duration in ms.
     */
    public FadeAnimation(String startColor, String endColor, long durationMs) {
        this.startColor = startColor;
        this.endColor = endColor;
        this.durationMs = durationMs;

        // Precompute HSV values
        int startInt = (int) Long.parseLong(startColor.startsWith("#") ? startColor.substring(1) : startColor, 16);
        int endInt = (int) Long.parseLong(endColor.startsWith("#") ? endColor.substring(1) : endColor, 16);
        Color.colorToHSV(startInt, startHsv);
        Color.colorToHSV(endInt, endHsv);
    }

    @Override
    public String getColor(long elapsedTimeMs) {
        float t = Math.min((float) elapsedTimeMs / durationMs, 1.0f);

        // Interpolate HSV
        currentHsv[0] = startHsv[0] + t * (endHsv[0] - startHsv[0]);
        currentHsv[1] = startHsv[1] + t * (endHsv[1] - startHsv[1]);
        currentHsv[2] = startHsv[2] + t * (endHsv[2] - startHsv[2]);

        int color = Color.HSVToColor(currentHsv);
        return String.format("#%06X", color & 0xFFFFFF);
    }

    @Override
    public boolean isFinished(long elapsedTimeMs) {
        return elapsedTimeMs >= durationMs;
    }

    @Override
    public void reset() {
        // No state to reset
    }
}