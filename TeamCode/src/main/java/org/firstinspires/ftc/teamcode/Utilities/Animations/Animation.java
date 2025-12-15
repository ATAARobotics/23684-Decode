package org.firstinspires.ftc.teamcode.Utilities.Animations;

/**
 * Interface for RGB Indicator animations.
 * Animations maintain internal state and update over time.
 */
public interface Animation {
    /**
     * Updates the animation state by the given delta time.
     * @param deltaTimeMs Time in milliseconds since last update.
     */
    void update(long deltaTimeMs);

    /**
     * Gets the current color for the animation.
     * @return Hex color string (e.g., "#FF0000").
     */
    String getColor();

    /**
     * Checks if the animation has finished.
     * @return True if animation is complete, false otherwise.
     */
    boolean isFinished();

    /**
     * Resets the animation to its initial state.
     */
    void reset();
}