package org.firstinspires.ftc.teamcode.Utilities.Animations;

/**
 * Animation that blinks between two colors at a specified interval.
 */
public class BlinkAnimation implements Animation {
    private final String color1;
    private final String color2;
    private final long intervalMs;
    private long elapsedTimeMs = 0;

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
    public void update(long deltaTimeMs) {
        elapsedTimeMs += deltaTimeMs;
    }

    @Override
    public String getColor() {
        long phase = (elapsedTimeMs / intervalMs) % 2;
        return phase == 0 ? color1 : color2;
    }

    @Override
    public boolean isFinished() {
        return false; // Infinite blink
    }

    @Override
    public void reset() {
        elapsedTimeMs = 0;
    }
}