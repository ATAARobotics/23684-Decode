package org.firstinspires.ftc.teamcode.OpModes.Auto.Modular;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Utils.Team;

public class PoseDatabase {
	// BLUE POSES
	public static final Pose BLUE_START = new Pose(63.000, 9, Math.toRadians(270));
	public static final Pose BLUE_SHOOT = new Pose(59.440, 17.328, Math.toRadians(294.935));

	public static final Pose BLUE_AUDIENCE_SHOOT = new Pose(91.328, 17.417, Math.toRadians(-56.535));
	public static final Pose BLUE_GOAL_SHOOT = new Pose(120.4, 95.7, Math.toRadians(340.7));

	public static final Pose BLUE_SPIKE_1_INTERMEDIATE = new Pose(46.000, 45.50, Math.toRadians(180));
	public static final Pose BLUE_SPIKE_1_COLLECT = new Pose(21.000, 45.50, Math.toRadians(180));

	public static final Pose BLUE_SPIKE_2_INTERMEDIATE = new Pose(46.000, 67.500, Math.toRadians(180));
	public static final Pose BLUE_SPIKE_2_COLLECT = new Pose(20.000, 67.500, Math.toRadians(180));

	public static final Pose BLUE_SPIKE_3_INTERMEDIATE = new Pose(46.000, 94.000, Math.toRadians(180));
	public static final Pose BLUE_SPIKE_3_COLLECT = new Pose(25.000, 94.000, Math.toRadians(180));

	public static final Pose BLUE_HUMAN_PLAYER_INTERMEDIATE = new Pose(44.8, 19.128, Math.toRadians(180));
	public static final Pose BLUE_HUMAN_PLAYER_COLLECT = new Pose(17.00, 19.128, Math.toRadians(180));

	public static final Pose BLUE_HUMAN_PLAYER_COLLECT_FARSIDE = new Pose(44.00, 45.50, Math.toRadians(180));
	public static final Pose BLUE_HUMAN_PLAYER_COLLECT_WIGGLE = new Pose(15.8, 19.128, Math.toRadians(180));

	public static final Pose BLUE_PARK = new Pose(40, 34, Math.toRadians(180));
	public static final Pose BLUE_RESET_POSE = new Pose(142.720, 7.368, 0);

	// RED POSES
	public static final Pose RED_START = new Pose(81.000, 9, Math.toRadians(-90));
	public static final Pose RED_SHOOT = new Pose(83.000, 17.328, Math.toRadians(-114.14));

	public static final Pose RED_AUDIENCE_SHOOT = new Pose(51.639, 22.730, Math.toRadians(-122.851));
	public static final Pose RED_GOAL_SHOOT = new Pose(39.03, 101.80, Math.toRadians(203));

	public static final Pose RED_SPIKE_1_INTERMEDIATE = new Pose(98.000, 34.500, Math.toRadians(0));
	public static final Pose RED_SPIKE_1_COLLECT = new Pose(136.000, 34.500, Math.toRadians(0));

	public static final Pose RED_SPIKE_2_INTERMEDIATE = new Pose(98.000, 58.000, Math.toRadians(0));
	public static final Pose RED_SPIKE_2_COLLECT = new Pose(136.000, 58.000, Math.toRadians(0));

	public static final Pose RED_SPIKE_3_INTERMEDIATE = new Pose(98.000, 82.000, Math.toRadians(0));
	public static final Pose RED_SPIKE_3_COLLECT = new Pose(123.000, 82.000, Math.toRadians(0));

	public static final Pose RED_HUMAN_PLAYER_INTERMEDIATE = new Pose(120.000, 9.128, Math.toRadians(0));
	public static final Pose RED_HUMAN_PLAYER_COLLECT = new Pose(136.000, 9.128, Math.toRadians(0));
	public static final Pose RED_HUMAN_PLAYER_COLLECT_WIGGLE = new Pose(137.5, 9.128, Math.toRadians(0));

	public static final Pose RED_HUMAN_PLAYER_COLLECT_FARSIDE = new Pose(120.000, 34.500, Math.toRadians(0));
	public static final Pose RED_HUMAN_PLAYER_COLLECT_WIGGLE_CLOSE = new Pose(139.5, 9.128, Math.toRadians(0));


	public static final Pose RED_PARK = new Pose(105, 34, Math.toRadians(0));
	public static final Pose RED_RESET_POSE = new Pose(7.1, 18, Math.toRadians(180));

	public static Pose getShootPose(Team team) {
		return team == Team.BLUE ? BLUE_SHOOT : RED_SHOOT;
	}

	public static Pose getStartPose(Team team) {
		return team == Team.BLUE ? BLUE_START : RED_START;
	}

	public static Pose getAudienceShootPose(Team team) {
		return team == Team.BLUE ? BLUE_AUDIENCE_SHOOT : RED_AUDIENCE_SHOOT;
	}

	public static Pose getGoalShootPose(Team team) {
		return team == Team.BLUE ? BLUE_GOAL_SHOOT : RED_GOAL_SHOOT;
	}

	public static Pose getResetPose(Team team) {
		return team == Team.BLUE ? BLUE_RESET_POSE : RED_RESET_POSE;
	}
}
