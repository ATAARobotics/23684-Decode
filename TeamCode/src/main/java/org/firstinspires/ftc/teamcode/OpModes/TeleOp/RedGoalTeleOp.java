package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utils.Team;

@TeleOp(name = "Red Goal TeleOp", group = "Red")
public class RedGoalTeleOp extends MainTeleOp {
	@Override
	protected Pose getStartingPose() {
		// Mirrored X for Red Alliance, Goal side
		return new Pose(144 - 63.450, 135, Math.toRadians(90));
	}

	@Override
	protected Team getTeam() {
		return Team.RED;
	}
}
