package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Utils.Team;

@Disabled
@Autonomous(name = "Blue Human Player + 2 spike marks", group = "Blue", preselectTeleOp = "Blue TeleOp")
public class BlueHuman extends AudienceHumanAuto {

	@Override
	protected Pose getStartingPose() {
		return new Pose(63.450, 9, Math.toRadians(270));
	}

	@Override
	protected Team getTeam() {
		return Team.BLUE;
	}
}