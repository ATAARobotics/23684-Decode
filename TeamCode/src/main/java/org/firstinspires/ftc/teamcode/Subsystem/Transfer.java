package org.firstinspires.ftc.teamcode.Subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import java.util.Set;

@Configurable
public class Transfer extends SubsystemBase {
    public static double SHOOTER_RPM_TOLERANCE = 150;
    
    public final CRServo transferLeft;
	public final CRServo transferRight;
	public final CRServo intakeDoorRight;
	public final CRServo intakeDoorLeft;
	private Shooter shooter;
	public boolean isTransferOutActive = false;
	public boolean reachedUpperTarget;
	public boolean reachedLowerTarget;
	public boolean reachedAverageTarget;

    public Transfer(HardwareMap hardwareMap) {
        transferLeft = hardwareMap.get(CRServo.class, "transferLeft");
        transferRight = hardwareMap.get(CRServo.class, "transferRight");
        transferRight.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeDoorRight = hardwareMap.get(CRServo.class, "intakeDoorRight");
        intakeDoorLeft = hardwareMap.get(CRServo.class, "intakeDoorLeft");
        intakeDoorRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setShooter(Shooter shooter) {
        this.shooter = shooter;
    }

    public Command IntakeDoorIn(){
        return new InstantCommand( () -> {
            intakeDoorLeft.setPower(1);
            intakeDoorRight.setPower(1);
        }, this
        );
    }

    public Command IntakeDoorOut(){
        return new InstantCommand( () -> {
            intakeDoorLeft.setPower(-1);
            intakeDoorRight.setPower(-1);
        }, this
        );
    }

    public Command IntakeDoorStop(){
        return new InstantCommand( () -> {
            intakeDoorLeft.setPower(0);
            intakeDoorRight.setPower(0);
        }, this
        );
    }

   public Command TransferIn(){
       return new InstantCommand( () -> {
           transferLeft.setPower(-1);
           transferRight.setPower(-1);
       }, this
       );
    }

    public Command TransferOut(){
        return new InstantCommand( () -> {
            transferLeft.setPower(1);
            transferRight.setPower(1);
        }, this
        );
    }

    public Command TransferStop(){
        return new InstantCommand( () -> {
            transferLeft.setPower(0);
            transferRight.setPower(0);
        }, this
        );
    }

    public boolean isShooterReady(double shooterRpm, double targetRpm) {
        return Math.abs(shooterRpm - targetRpm) <= SHOOTER_RPM_TOLERANCE;
    }

    public void updateConditionalTransferOut() {
        if (isTransferOutActive) {
            if (shooter != null){
//				reachedUpperTarget = (shooter.upperTarget >= 100) && (Math.abs(shooter.upperRPM - shooter.upperTarget) <= SHOOTER_RPM_TOLERANCE);
//				reachedLowerTarget = (shooter.lowerTarget >= 100) && (Math.abs(shooter.lowerRPM - shooter.lowerTarget) <= SHOOTER_RPM_TOLERANCE);
				reachedAverageTarget = (shooter.lowerTarget >= 100) && (shooter.upperTarget >= 100) && (((Math.abs(shooter.lowerRPM - shooter.lowerTarget) + Math.abs(shooter.upperRPM - shooter.upperTarget)) / 2) <= SHOOTER_RPM_TOLERANCE);

                if (reachedAverageTarget) {
					transferLeft.setPower(1);
                	transferRight.setPower(1);
				} else {
					transferLeft.setPower(-1);
					transferRight.setPower(-1);
				}
			}
        }
	}
}
