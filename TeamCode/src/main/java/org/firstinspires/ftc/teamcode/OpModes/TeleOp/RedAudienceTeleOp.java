package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utils.Team;

@TeleOp(name = "Red Audience TeleOp", group = " 1COMP")
public class RedAudienceTeleOp extends MainTeleOp {
	@Override
	protected Pose getStartingPose() {
		// Mirrored X for Red Alliance
		return new Pose(81.00, 9, Math.toRadians(270));
	}

	@Override
	protected Team getTeam() {
		return Team.RED;
	}
}
