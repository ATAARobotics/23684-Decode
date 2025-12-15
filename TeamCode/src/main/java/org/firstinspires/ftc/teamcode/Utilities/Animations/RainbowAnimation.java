package org.firstinspires.ftc.teamcode.Utilities.Animations;

import android.graphics.Color;

/**
 * Animation that cycles through the rainbow colors.
 */
public class RainbowAnimation implements Animation {
    private final long cycleDurationMs;
    private final float[] hsv = new float[3];

    /**
     * @param cycleDurationMs Duration for one full rainbow cycle in ms.
     */
    public RainbowAnimation(long cycleDurationMs) {
        this.cycleDurationMs = cycleDurationMs;
        hsv[1] = 1.0f; // Full saturation
        hsv[2] = 1.0f; // Full value
    }

    @Override
    public String getColor(long elapsedTimeMs) {
        // Hue cycles from 0 to 360 over the cycle duration
        float hue = (elapsedTimeMs % cycleDurationMs) / (float) cycleDurationMs * 360.0f;
        hsv[0] = hue;

        int color = Color.HSVToColor(hsv);
        return String.format("#%06X", color & 0xFFFFFF);
    }

    @Override
    public boolean isFinished(long elapsedTimeMs) {
        return false; // Infinite rainbow
    }

    @Override
    public void reset() {
        // No state to reset
    }
}