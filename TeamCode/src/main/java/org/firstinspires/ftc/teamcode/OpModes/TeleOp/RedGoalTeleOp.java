package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utils.Team;


@Disabled
@TeleOp(name = "Red TeleOp", group = " 1COMP")
public class RedGoalTeleOp extends MainTeleOp {
	@Override
	protected Pose getStartingPose() {
		// Mirrored X for Red Alliance, Goal side
		return new Pose(80.25, 135, Math.toRadians(90));
	}

	@Override
	protected Team getTeam() {
		return Team.RED;
	}
}
