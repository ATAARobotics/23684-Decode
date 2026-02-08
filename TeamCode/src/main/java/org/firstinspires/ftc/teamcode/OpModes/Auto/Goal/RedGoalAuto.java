package org.firstinspires.ftc.teamcode.OpModes.Auto.Goal;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Utils.Team;

@Autonomous(name = "Red Goal Auto", group = "Red")
public class RedGoalAuto extends GoalAuto {
	@Override
	protected Pose getStartingPose() {
		return new Pose(122.693, 124.098, Math.toRadians(-144));
	}

	@Override
	protected Team getTeam() {
		return Team.RED;
	}
}
