package org.firstinspires.ftc.teamcode.OpModes.Auto.OLD;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Utils.Team;


@Disabled
@Autonomous(name = "Blue Audience Auto", group = "Blue", preselectTeleOp = "Blue TeleOp")
public class BlueAudienceAuto extends AudienceAuto {
	@Override
	protected Pose getStartingPose() {
		return new Pose(63.450, 9, Math.toRadians(270));
	}

	@Override
	protected Team getTeam() {
		return Team.BLUE;
	}
}