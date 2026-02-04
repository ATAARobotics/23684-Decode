package org.firstinspires.ftc.teamcode.Utils;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;
import org.firstinspires.ftc.teamcode.Subsystem.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystem.Transfer;

public class ShootArtifacts extends SequentialCommandGroup {
	public ShootArtifacts(Shooter shooter, Spindexer spindexer, Transfer transfer, Intake intake) {
		addCommands(
				shooter.SetTarget(Shooter.AUDIENCE_RPM, Shooter.AUDIENCE_RPM),
				shooter.WaitForTarget().withTimeout(3000L),
				transfer.TransferOut(),
				spindexer.DirectPower(0.5),
				shooter.WaitForDrop().withTimeout(2500),
				shooter.WaitForTarget().withTimeout(2500),
				shooter.WaitForDrop().withTimeout(2500),
				shooter.WaitForTarget().withTimeout(2500),
				shooter.WaitForDrop().withTimeout(2500),
				transfer.TransferStop(),
				shooter.SetTarget(0, 0)
		);

		addRequirements(shooter, spindexer, transfer, intake);
	}
}