package org.firstinspires.ftc.teamcode.Utilities;

/**
 * Utility class for calculating shot angles based on robot and goal positions
 */
public class ShotAngle {
    /**
     * Calculate the angle needed to aim at a goal position from the robot's current position
     *
     * @param x     Robot's current X position
     * @param y     Robot's current Y position
     * @param goalX Goal's X position
     * @param goalY Goal's Y position
     * @return The angle in radians needed to aim at the goal
     */
    public static double calculateShotAngle(double x, double y, double goalX, double goalY) {
        double deltaX = goalX - x;
        double deltaY = goalY - y;
        return Math.atan2(-deltaY, -deltaX);
    }
}
