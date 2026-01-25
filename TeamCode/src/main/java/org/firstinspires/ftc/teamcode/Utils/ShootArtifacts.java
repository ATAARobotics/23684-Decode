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
				transfer.SetAutomaticTransfer(true),

				// Cycle 1
				shooter.SetTarget(Shooter.AUDIENCE_RPM, Shooter.AUDIENCE_RPM),
				shooter.WaitForTarget(),
				shooter.WaitForDrop(),
				new WaitCommand(300),

				// Cycle 2
				spindexer.NextTarget(),
				shooter.WaitForTarget(),
				shooter.WaitForDrop(),
				new WaitCommand(300),

				// Cycle 3
				spindexer.NextTarget(),
				shooter.WaitForTarget(),
				shooter.WaitForDrop(),
				new WaitCommand(300),

				transfer.SetAutomaticTransfer(false)
		);

		addRequirements(shooter, spindexer, transfer, intake);
	}
}