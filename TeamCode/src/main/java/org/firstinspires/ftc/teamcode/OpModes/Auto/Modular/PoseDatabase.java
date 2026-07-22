package org.firstinspires.ftc.teamcode.OpModes.Auto.Modular;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Utils.PoseUtils;
import org.firstinspires.ftc.teamcode.Utils.Team;

public class PoseDatabase {
	// BLUE POSES  — source of truth for the entire alliance pair.
	// Every RED_* pose below is PoseUtils.mirror(BLUE_*, FIELD_LENGTH) where FIELD_LENGTH = 141.5".
	public static final Pose BLUE_START = new Pose(63.000, 9, Math.toRadians(270));
	public static final Pose BLUE_SHOOT = new Pose(59.440, 17.328, Math.toRadians(294.935));

	public static final Pose BLUE_AUDIENCE_SHOOT = new Pose(91.328, 17.417, Math.toRadians(-56.535));
	public static final Pose BLUE_GOAL_SHOOT = new Pose(120.4, 95.7, Math.toRadians(340.7));

	public static final Pose BLUE_SPIKE_1_INTERMEDIATE = new Pose(46.000, 35.5, Math.toRadians(180));
	public static final Pose BLUE_SPIKE_1_COLLECT = new Pose(21.000, 35.5, Math.toRadians(180));

	public static final Pose BLUE_SPIKE_2_INTERMEDIATE = new Pose(46.000, 59.7, Math.toRadians(180));
	public static final Pose BLUE_SPIKE_2_COLLECT = new Pose(20.000, 59.7, Math.toRadians(180));

	public static final Pose BLUE_SPIKE_3_INTERMEDIATE = new Pose(46.000, 83.87, Math.toRadians(180));
	public static final Pose BLUE_SPIKE_3_COLLECT = new Pose(25.000, 83.87, Math.toRadians(180));

	public static final Pose BLUE_HUMAN_PLAYER_INTERMEDIATE = new Pose(44.8, 10.5, Math.toRadians(180));
	public static final Pose BLUE_HUMAN_PLAYER_COLLECT = new Pose(17.00, 10.5, Math.toRadians(180));

	public static final Pose BLUE_HUMAN_PLAYER_COLLECT_FARSIDE = new Pose(44.00, 35.5, Math.toRadians(180));

	public static final Pose BLUE_PARK = new Pose(38.5, 33, Math.toRadians(180));
	public static final Pose BLUE_RESET_POSE = new Pose(142.720, 7.368, 0);

	// RED POSES  — every entry below is exactly PoseUtils.mirror(BLUE_*, 141.5).
	// Field: 141.5" x 141.5" (custom). Mirror rule: (x, y, θ) → (141.5 − x, y, −θ).
	// If you need a red-specific tweak, add it as a NEW blue pose and mirror it,
	// not by editing a red constant — that breaks the symmetry invariant.
	public static final double FIELD_LENGTH = 141.5;

	public static final Pose RED_START = PoseUtils.mirror(BLUE_START, FIELD_LENGTH);
	public static final Pose RED_SHOOT = PoseUtils.mirror(BLUE_SHOOT, FIELD_LENGTH);

	public static final Pose RED_AUDIENCE_SHOOT = PoseUtils.mirror(BLUE_AUDIENCE_SHOOT, FIELD_LENGTH);
	public static final Pose RED_GOAL_SHOOT = PoseUtils.mirror(BLUE_GOAL_SHOOT, FIELD_LENGTH);

	public static final Pose RED_SPIKE_1_INTERMEDIATE = PoseUtils.mirror(BLUE_SPIKE_1_INTERMEDIATE, FIELD_LENGTH);
	public static final Pose RED_SPIKE_1_COLLECT = PoseUtils.mirror(BLUE_SPIKE_1_COLLECT, FIELD_LENGTH);

	public static final Pose RED_SPIKE_2_INTERMEDIATE = PoseUtils.mirror(BLUE_SPIKE_2_INTERMEDIATE, FIELD_LENGTH);
	public static final Pose RED_SPIKE_2_COLLECT = PoseUtils.mirror(BLUE_SPIKE_2_COLLECT, FIELD_LENGTH);

	public static final Pose RED_SPIKE_3_INTERMEDIATE = PoseUtils.mirror(BLUE_SPIKE_3_INTERMEDIATE, FIELD_LENGTH);
	public static final Pose RED_SPIKE_3_COLLECT = PoseUtils.mirror(BLUE_SPIKE_3_COLLECT, FIELD_LENGTH);

	public static final Pose RED_HUMAN_PLAYER_INTERMEDIATE = PoseUtils.mirror(BLUE_HUMAN_PLAYER_INTERMEDIATE, FIELD_LENGTH);
	public static final Pose RED_HUMAN_PLAYER_COLLECT = PoseUtils.mirror(BLUE_HUMAN_PLAYER_COLLECT, FIELD_LENGTH);
	public static final Pose RED_HUMAN_PLAYER_COLLECT_FARSIDE = PoseUtils.mirror(BLUE_HUMAN_PLAYER_COLLECT_FARSIDE, FIELD_LENGTH);

	public static final Pose RED_PARK = PoseUtils.mirror(BLUE_PARK, FIELD_LENGTH);
	public static final Pose RED_RESET_POSE = PoseUtils.mirror(BLUE_RESET_POSE, FIELD_LENGTH);

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
