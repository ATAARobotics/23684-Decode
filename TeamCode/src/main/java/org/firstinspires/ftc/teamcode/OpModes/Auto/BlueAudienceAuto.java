package org.firstinspires.ftc.teamcode.OpModes.Auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.Utils.Team;

@Autonomous (name = "Blue Audience Auto", group = "Blue")
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