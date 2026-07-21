package org.firstinspires.ftc.teamcode.OpModes.Auto.Modular;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Utils.Team;

public class PoseDatabase {
	// BLUE POSES  — source of truth for the entire alliance pair.
	// Every RED_* pose below is BLUE_*.mirror(FIELD_LENGTH) where FIELD_LENGTH = 141.5".
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

	// RED POSES  — every entry below is exactly BLUE_*.mirror(141.5).
	// Field: 141.5" x 141.5" (custom). Mirror rule: (x, y, θ) → (141.5 − x, y, −θ).
	// If you need a red-specific tweak, add it as a NEW blue pose and mirror it,
	// not by editing a red constant — that breaks the symmetry invariant.
	public static final double FIELD_LENGTH = 141.5;

	public static final Pose RED_START = BLUE_START.mirror(FIELD_LENGTH);
	public static final Pose RED_SHOOT = BLUE_SHOOT.mirror(FIELD_LENGTH);

	public static final Pose RED_AUDIENCE_SHOOT = BLUE_AUDIENCE_SHOOT.mirror(FIELD_LENGTH);
	public static final Pose RED_GOAL_SHOOT = BLUE_GOAL_SHOOT.mirror(FIELD_LENGTH);

	public static final Pose RED_SPIKE_1_INTERMEDIATE = BLUE_SPIKE_1_INTERMEDIATE.mirror(FIELD_LENGTH);
	public static final Pose RED_SPIKE_1_COLLECT = BLUE_SPIKE_1_COLLECT.mirror(FIELD_LENGTH);

	public static final Pose RED_SPIKE_2_INTERMEDIATE = BLUE_SPIKE_2_INTERMEDIATE.mirror(FIELD_LENGTH);
	public static final Pose RED_SPIKE_2_COLLECT = BLUE_SPIKE_2_COLLECT.mirror(FIELD_LENGTH);

	public static final Pose RED_SPIKE_3_INTERMEDIATE = BLUE_SPIKE_3_INTERMEDIATE.mirror(FIELD_LENGTH);
	public static final Pose RED_SPIKE_3_COLLECT = BLUE_SPIKE_3_COLLECT.mirror(FIELD_LENGTH);

	public static final Pose RED_HUMAN_PLAYER_INTERMEDIATE = BLUE_HUMAN_PLAYER_INTERMEDIATE.mirror(FIELD_LENGTH);
	public static final Pose RED_HUMAN_PLAYER_COLLECT = BLUE_HUMAN_PLAYER_COLLECT.mirror(FIELD_LENGTH);
	public static final Pose RED_HUMAN_PLAYER_COLLECT_FARSIDE = BLUE_HUMAN_PLAYER_COLLECT_FARSIDE.mirror(FIELD_LENGTH);
	public static final Pose RED_HUMAN_PLAYER_COLLECT_WIGGLE = BLUE_HUMAN_PLAYER_COLLECT_WIGGLE.mirror(FIELD_LENGTH);

	public static final Pose RED_PARK = BLUE_PARK.mirror(FIELD_LENGTH);
	public static final Pose RED_RESET_POSE = BLUE_RESET_POSE.mirror(FIELD_LENGTH);

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
