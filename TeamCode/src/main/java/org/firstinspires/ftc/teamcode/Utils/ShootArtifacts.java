package org.firstinspires.ftc.teamcode.Utils;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;
import org.firstinspires.ftc.teamcode.Subsystem.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystem.Touch;
import org.firstinspires.ftc.teamcode.Subsystem.Transfer;

public class ShootArtifacts extends SequentialCommandGroup {
	public ShootArtifacts(Shooter shooter, Spindexer spindexer, Transfer transfer, Intake intake, Touch touch) {
		addCommands(
				shooter.SetTarget(Shooter.AUDIENCE_RPM , Shooter.AUDIENCE_RPM),
				shooter.WaitForTarget().withTimeout(3000L),
				spindexer.DirectPower(0.25),
				touch.ShootAllArtifacts(),
//				shooter.WaitForDrop().withTimeout(560),
//				shooter.WaitForTarget().withTimeout(560),
//				shooter.WaitForDrop().withTimeout(560),
//				shooter.WaitForTarget().withTimeout(560),
//				shooter.WaitForDrop().withTimeout(560),
				spindexer.DirectPower(0),
				touch.ResetCounter()
		);

		addRequirements(shooter, spindexer, transfer, intake);
	}
}