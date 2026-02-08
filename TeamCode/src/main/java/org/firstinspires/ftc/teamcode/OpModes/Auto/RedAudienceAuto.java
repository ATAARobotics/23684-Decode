package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Utils.Team;

@Autonomous(name = "Red Audience Auto", group = "Red", preselectTeleOp = "Red Audience TeleOp")
public class RedAudienceAuto extends AudienceAuto {
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
