package org.firstinspires.ftc.teamcode.OpModes.Auto.Modular;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Utils.Team;

@Autonomous(name = "Blue Audience Auto", group = "Blue", preselectTeleOp = "Blue TeleOp")
public class BlueModularAudienceAuto extends ModularAuto {
	@Override
	protected Pose getStartingPose() {
		return PoseDatabase.BLUE_START;
	}

	@Override
	protected Team getTeam() {
		return Team.BLUE;
	}

	@Override
	protected void setRoute() {
		addStep(RouteStep.SHOOT_PRELOAD);
		addStep(RouteStep.COLLECT_SPIKE_1);
		addStep(RouteStep.SHOOT);
		addStep(RouteStep.COLLECT_SPIKE_2);
		addStep(RouteStep.SHOOT);
		addStep(RouteStep.COLLECT_SPIKE_3);
		addStep(RouteStep.SHOOT);
		addStep(RouteStep.COLLECT_HUMAN_PLAYER);
		addStep(RouteStep.SHOOT_WITH_BEAMBREAKER);
		addStep(RouteStep.COLLECT_HUMAN_PLAYER);
		addStep(RouteStep.SHOOT_WITH_BEAMBREAKER);
		addStep(RouteStep.PARK);
	}
}
