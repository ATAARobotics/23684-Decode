package org.firstinspires.ftc.teamcode.Utils;

import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Subsystem.Conveyor;
import org.firstinspires.ftc.teamcode.Subsystem.Gate;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;
import org.firstinspires.ftc.teamcode.Subsystem.Transfer;

public class ShootArtifacts extends SequentialCommandGroup {
	public ShootArtifacts(Shooter shooter, Conveyor conveyor, Transfer transfer, Intake intake, Gate gate, int waitTime) {
		addCommands(
				shooter.SetTarget(Shooter.AUDIENCE_RPM_UPPER, Shooter.AUDIENCE_RPM_LOWER),
				new ParallelCommandGroup(
						intake.Stop(),
						new SequentialCommandGroup(
							new WaitUntilCommand(() -> shooter.getPercentToTarget() >= 0.8),
							gate.openGate()
						),
				shooter.WaitForTarget().withTimeout(2500L)
				),
				transfer.TransferOut(),
				conveyor.In(),
				new WaitCommand(waitTime),
				conveyor.Stop(),
				gate.closeGate()
		);

		addRequirements(shooter, conveyor, transfer, intake);
	}
}
