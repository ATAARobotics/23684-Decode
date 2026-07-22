package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utils.Team;


@Disabled
@TeleOp(name = "Blue Goal TeleOp", group = " 1COMP")
public class BlueGoalTeleOp extends MainTeleOp {
	@Override
	protected Pose getStartingPose() {
		return new Pose(61.25, 135, Math.toRadians(90));
	}

	@Override
	protected Team getTeam() {
		return Team.BLUE;
	}
}