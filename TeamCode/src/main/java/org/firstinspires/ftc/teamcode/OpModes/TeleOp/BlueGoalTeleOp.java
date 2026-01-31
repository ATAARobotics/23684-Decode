package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utils.Team;

@TeleOp(name = "Blue Goal TeleOp", group = "Blue")
public class BlueGoalTeleOp extends MainTeleOp {
	@Override
	protected Pose getStartingPose() {
		return new Pose(63.450, 135, Math.toRadians(90));
	}

	@Override
	protected Team getTeam() {
		return Team.BLUE;
	}
}