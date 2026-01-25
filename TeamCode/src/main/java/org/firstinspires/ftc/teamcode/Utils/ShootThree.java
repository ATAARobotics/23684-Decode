package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;
import org.firstinspires.ftc.teamcode.Subsystem.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystem.Transfer;

public class ShootThree extends CommandBase {
    Shooter shooter;
    Spindexer spindexer;
    Intake intake;
    Transfer transfer;

    public ShootThree(Shooter shooter, Spindexer spindexer, Transfer transfer, Intake intake) {
        this.shooter = shooter;
		this.spindexer = spindexer;
		this.transfer = transfer;
		this.intake = intake;

		addRequirements(this.shooter, this.shooter, this.transfer, this.intake);
    }

    public Command ShootThreeArtifacts() {
        return new SequentialCommandGroup(
                transfer.TransferIn(),
                new ParallelCommandGroup(
                    shooter.SetTarget(Shooter.AUDIENCE_RPM, Shooter.AUDIENCE_RPM),
                    shooter.WaitForTarget()
                ),
               transfer.TransferOut(),

               new WaitCommand(300),
               transfer.TransferIn(),
//               spindexer.NextSlot(),
               new ParallelCommandGroup(
                        shooter.SetTarget(Shooter.AUDIENCE_RPM, Shooter.AUDIENCE_RPM),
                        shooter.WaitForTarget()
               ),
              transfer.TransferOut(),

              new WaitCommand(300),
              transfer.TransferIn(),
//              spindexer.NextSlot(),
              new ParallelCommandGroup(
                      shooter.SetTarget(Shooter.AUDIENCE_RPM, Shooter.AUDIENCE_RPM),
                      shooter.WaitForTarget()
              ),
              transfer.TransferOut()
        );
    }



}
