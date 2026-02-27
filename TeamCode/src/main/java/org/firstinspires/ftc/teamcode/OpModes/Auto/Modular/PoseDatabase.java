package org.firstinspires.ftc.teamcode.OpModes.Auto.Modular;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.Utils.Team;

public class PoseDatabase {
    // BLUE POSES
    public static final Pose BLUE_START = new Pose(63.000, 9, Math.toRadians(270));
    public static final Pose BLUE_SHOOT = new Pose(59.440, 17.328, Math.toRadians(294.935));
    
    public static final Pose BLUE_SPIKE_1_INTERMEDIATE = new Pose(46.000, 40.50, Math.toRadians(180));
    public static final Pose BLUE_SPIKE_1_COLLECT = new Pose(11.000, 40.50, Math.toRadians(180));
    
    public static final Pose BLUE_SPIKE_2_INTERMEDIATE = new Pose(46.000, 65.500, Math.toRadians(180));
    public static final Pose BLUE_SPIKE_2_COLLECT = new Pose(11.000, 65.500, Math.toRadians(180));
    
    public static final Pose BLUE_SPIKE_3_INTERMEDIATE = new Pose(46.000, 89.500, Math.toRadians(180));
    public static final Pose BLUE_SPIKE_3_COLLECT = new Pose(15.000, 89.500, Math.toRadians(180));
    
    public static final Pose BLUE_HUMAN_PLAYER_INTERMEDIATE = new Pose(46, 20.328, Math.toRadians(180));
    public static final Pose BLUE_HUMAN_PLAYER_COLLECT = new Pose(24.000, 20.328, Math.toRadians(180));
    public static final Pose BLUE_HUMAN_PLAYER_COLLECT_WIGGLE = new Pose(17, 20.328, Math.toRadians(180));

    public static final Pose BLUE_PARK = new Pose(40, 34, Math.toRadians(180));

    // RED POSES
    public static final Pose RED_START = new Pose(81.000, 9, Math.toRadians(-90));
    public static final Pose RED_SHOOT = new Pose(83.000, 11.000, Math.toRadians(-114.14));
    
    public static final Pose RED_SPIKE_1_INTERMEDIATE = new Pose(98.000, 35.000, Math.toRadians(0));
    public static final Pose RED_SPIKE_1_COLLECT = new Pose(143.000, 35.000, Math.toRadians(0));
    
    public static final Pose RED_SPIKE_2_INTERMEDIATE = new Pose(98.000, 60.000, Math.toRadians(0));
    public static final Pose RED_SPIKE_2_COLLECT = new Pose(143.000, 60.000, Math.toRadians(0));
    
    public static final Pose RED_SPIKE_3_INTERMEDIATE = new Pose(98.000, 84.000, Math.toRadians(0));
    public static final Pose RED_SPIKE_3_COLLECT = new Pose(129.000, 84.000, Math.toRadians(0));
    
    public static final Pose RED_HUMAN_PLAYER_INTERMEDIATE = new Pose(109, 20.328, Math.toRadians(0));
    public static final Pose RED_HUMAN_PLAYER_COLLECT = new Pose(132, 20.328, Math.toRadians(0));
    public static final Pose RED_HUMAN_PLAYER_COLLECT_WIGGLE = new Pose(136, 20.328, Math.toRadians(0));

    public static final Pose RED_PARK = new Pose(105, 34, Math.toRadians(0));

    public static Pose getShootPose(Team team) {
        return team == Team.BLUE ? BLUE_SHOOT : RED_SHOOT;
    }

    public static Pose getStartPose(Team team) {
        return team == Team.BLUE ? BLUE_START : RED_START;
    }
}
