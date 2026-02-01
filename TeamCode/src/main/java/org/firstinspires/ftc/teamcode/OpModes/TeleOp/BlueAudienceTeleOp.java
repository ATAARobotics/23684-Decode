package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utils.Team;

@TeleOp(name = "Blue Audience TeleOp", group = " 1COMP")
public class BlueAudienceTeleOp extends MainTeleOp {
	@Override
	protected Pose getStartingPose() {
		return new Pose(63.450, 9, Math.toRadians(270));
	}

	@Override
	protected Team getTeam() {
		return Team.BLUE;
	}
}
