package org.firstinspires.ftc.teamcode.Utilities.Animations;

/**
 * Animation that blinks between two colors at a specified interval.
 */
public class BlinkAnimation implements Animation {
    private final String color1;
    private final String color2;
    private final long intervalMs;

    /**
     * @param color1 First color (hex string).
     * @param color2 Second color (hex string).
     * @param intervalMs Time in ms for each blink phase.
     */
    public BlinkAnimation(String color1, String color2, long intervalMs) {
        this.color1 = color1;
        this.color2 = color2;
        this.intervalMs = intervalMs;
    }

    @Override
    public String getColor(long elapsedTimeMs) {
        long phase = (elapsedTimeMs / intervalMs) % 2;
        return phase == 0 ? color1 : color2;
    }

    @Override
    public boolean isFinished(long elapsedTimeMs) {
        return false; // Infinite blink
    }

    @Override
    public void reset() {
        // No state to reset
    }
}