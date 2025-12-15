package org.firstinspires.ftc.teamcode.Utilities.Animations;

/**
 * Animation that flashes between a color and off at a fast interval.
 */
public class StrobeAnimation implements Animation {
    private final String color;
    private final long intervalMs;

    /**
     * @param color Color to flash (hex string).
     * @param intervalMs Time in ms for each flash phase.
     */
    public StrobeAnimation(String color, long intervalMs) {
        this.color = color;
        this.intervalMs = intervalMs;
    }

    @Override
    public String getColor(long elapsedTimeMs) {
        long phase = (elapsedTimeMs / intervalMs) % 2;
        return phase == 0 ? color : "#000000"; // Off
    }

    @Override
    public boolean isFinished(long elapsedTimeMs) {
        return false; // Infinite strobe
    }

    @Override
    public void reset() {
        // No state to reset
    }
}