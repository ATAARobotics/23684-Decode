package org.firstinspires.ftc.teamcode.Utilities.Animations;

/**
 * Animation that plays a sequence of colors with timings, then finishes.
 */
public class SequenceAnimation implements Animation {
    private final String[] colors;
    private final long[] durationsMs;
    private long totalDuration = 0;
    private long elapsedTimeMs = 0;

    /**
     * @param colors Array of hex color strings.
     * @param durationsMs Array of durations for each color (must match colors length).
     */
    public SequenceAnimation(String[] colors, long[] durationsMs) {
        if (colors.length != durationsMs.length) {
            throw new IllegalArgumentException("Colors and durations must have the same length");
        }
        this.colors = colors.clone();
        this.durationsMs = durationsMs.clone();
        for (long d : durationsMs) {
            totalDuration += d;
        }
    }

    @Override
    public void update(long deltaTimeMs) {
        elapsedTimeMs += deltaTimeMs;
    }

    @Override
    public String getColor() {
        if (elapsedTimeMs >= totalDuration) {
            return colors[colors.length - 1];
        }
        long currentTime = 0;
        for (int i = 0; i < colors.length; i++) {
            currentTime += durationsMs[i];
            if (elapsedTimeMs < currentTime) {
                return colors[i];
            }
        }
        return colors[colors.length - 1];
    }

    @Override
    public boolean isFinished() {
        return elapsedTimeMs >= totalDuration;
    }

    @Override
    public void reset() {
        elapsedTimeMs = 0;
    }
}