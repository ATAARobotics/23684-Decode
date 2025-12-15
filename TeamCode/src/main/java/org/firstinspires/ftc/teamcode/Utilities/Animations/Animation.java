package org.firstinspires.ftc.teamcode.Utilities.Animations;

/**
 * Interface for RGB Indicator animations.
 * Animations compute colors based on elapsed time and can indicate completion.
 */
public interface Animation {
    /**
     * Gets the current color for the animation at the given elapsed time.
     * @param elapsedTimeMs Time in milliseconds since animation start.
     * @return Hex color string (e.g., "#FF0000").
     */
    String getColor(long elapsedTimeMs);

    /**
     * Checks if the animation has finished.
     * @param elapsedTimeMs Time in milliseconds since animation start.
     * @return True if animation is complete, false otherwise.
     */
    boolean isFinished(long elapsedTimeMs);

    /**
     * Resets the animation to its initial state.
     */
    void reset();
}