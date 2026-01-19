package org.firstinspires.ftc.teamcode.Utils;

/**
 * SpindexerPosition provides utility methods for calculating
 * the next positions for the Spindexer to shoot or intake balls.
 * The utility uses modular arithmetic to determine positions:
 * - Shoot positions: x ≡ 0 (mod 120)
 * - Intake positions: x ≡ 20 (mod 120)
 */
public class SpindexerPosition {

    /**
     * Calculates the next position for the Spindexer to shoot a ball.
     * The next shoot position is the smallest x > currentPosition where x ≡ 0 (mod 120).
     *
     * @param currentPosition the current position of the Spindexer
     * @return the next position to shoot a ball
     */
    public static int getNextShootPosition(int currentPosition) {
        int targetRemainder = 20;
        int remainder = currentPosition % 120;

        // If we're already at a shoot position (with tolerance of ±10 degrees)
        if (remainder >= targetRemainder - 10 && remainder <= targetRemainder + 10) {
            return currentPosition + 120;
        }
        // If we've passed the shoot position in this cycle
        else if (remainder > targetRemainder) {
            return currentPosition + (120 - remainder + targetRemainder);
        }

        // If we haven't reached the shoot position yet in this cycle
        else {
            return currentPosition + (targetRemainder - remainder);
        }
    }

    /**
     * Calculates the next position for the Spindexer to intake a ball.
     *
     * @param currentPosition the current position of the Spindexer
     * @return the next position to intake a ball
     */
    public static int getNextIntakePosition(int currentPosition) {
        int targetRemainder = 100;
        int remainder = currentPosition % 120;

        // If we're already at a intake position (with tolerance of ±10 degrees)
        if (remainder >= targetRemainder - 10 && remainder <= targetRemainder + 10) {
            return currentPosition + 120;
        }

        // If we've passed the intake position in this cycle
        else if (remainder > targetRemainder) {
            return currentPosition + (120 - remainder + targetRemainder);
        }

        // If we haven't reached the intake position yet in this cycle
        else {
            return currentPosition + (targetRemainder - remainder);
        }
    }
}