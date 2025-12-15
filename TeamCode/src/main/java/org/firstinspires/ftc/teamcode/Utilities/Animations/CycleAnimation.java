package org.firstinspires.ftc.teamcode.Utilities.Animations;

/**
 * Animation that cycles through a list of colors at specified intervals.
 */
public class CycleAnimation implements Animation {
    private final String[] colors;
    private final long[] intervalsMs;

    /**
     * @param colors Array of hex color strings.
     * @param intervalsMs Array of durations for each color (must match colors length).
     */
    public CycleAnimation(String[] colors, long[] intervalsMs) {
        if (colors.length != intervalsMs.length) {
            throw new IllegalArgumentException("Colors and intervals must have the same length");
        }
        this.colors = colors.clone();
        this.intervalsMs = intervalsMs.clone();
    }

    @Override
    public String getColor(long elapsedTimeMs) {
        long totalCycleTime = 0;
        for (long interval : intervalsMs) {
            totalCycleTime += interval;
        }

        long cycleTime = elapsedTimeMs % totalCycleTime;
        long currentTime = 0;
        for (int i = 0; i < colors.length; i++) {
            currentTime += intervalsMs[i];
            if (cycleTime < currentTime) {
                return colors[i];
            }
        }
        return colors[colors.length - 1]; // Fallback
    }

    @Override
    public boolean isFinished(long elapsedTimeMs) {
        return false; // Infinite cycle
    }

    @Override
    public void reset() {
        // No state to reset
    }
}