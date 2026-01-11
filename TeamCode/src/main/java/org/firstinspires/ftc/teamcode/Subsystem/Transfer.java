package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class Transfer extends SubsystemBase {
    public final CRServo transferLeft;
	public final CRServo transferRight;
	public final CRServo intakeDoorRight;
	public final CRServo intakeDoorLeft;

    public Transfer(HardwareMap hardwareMap) {
        transferLeft = hardwareMap.get(CRServo.class, "transferLeft");
        transferRight = hardwareMap.get(CRServo.class, "transferRight");
        transferRight.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeDoorRight = hardwareMap.get(CRServo.class, "intakeDoorRight");
        intakeDoorLeft = hardwareMap.get(CRServo.class, "intakeDoorLeft");
        intakeDoorRight.setDirection(DcMotorSimple.Direction.REVERSE);
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

}
