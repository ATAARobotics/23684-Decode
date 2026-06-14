package org.firstinspires.ftc.teamcode.OpModes.Auto.Modular;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Utils.Team;

@Autonomous(name = "Red Triple Human Auto", group = "Red", preselectTeleOp = "Red TeleOp")
public class RedOnlyHuman extends ModularAuto {
	@Override
	protected Pose getStartingPose() {
		return PoseDatabase.RED_START;
	}

	@Override
	protected Team getTeam() {
		return Team.RED;
	}

	@Override
	protected void setRoute() {
		addStep(RouteStep.SHOOT_PRELOAD);
		addStep(RouteStep.COLLECT_HUMAN_PLAYER_CLOSE_WIGGLE);
		addStep(RouteStep.SHOOT);
		addStep(2000);
		addStep(RouteStep.COLLECT_HUMAN_PLAYER_WIGGLE);
		addStep(RouteStep.SHOOT);
		addStep(RouteStep.COLLECT_HUMAN_PLAYER_CLOSE_WIGGLE);
		//addStep(RouteStep.SHOOT);
		addStep(RouteStep.PARK);
	}
}
