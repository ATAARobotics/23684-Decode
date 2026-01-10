package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;

public class Transfer {
    CRServo transferLeft;
    CRServo transferRight;
    CRServo intakeDoorRight;
    CRServo intakeDoorLeft;

    public Transfer(HardwareMap hardwareMap) {
        transferLeft = hardwareMap.get(CRServo.class, "transferLeft");
        transferRight = hardwareMap.get(CRServo.class, "transferRight");
        intakeDoorRight = hardwareMap.get(CRServo.class, "intakeDoorRight");
        intakeDoorLeft = hardwareMap.get(CRServo.class, "intakeDoorLeft");
    }

    public Command IntakeDoorIn(){
        return new InstantCommand( () -> {
            intakeDoorLeft.setPower(1);
            intakeDoorRight.setPower(1);
        }
        );
    }

    public Command IntakeDoorOut(){
        return new InstantCommand( () -> {
            intakeDoorLeft.setPower(-1);
            intakeDoorRight.setPower(-1);
        }
        );
    }

   public Command TransferIn(){
       return new InstantCommand( () -> {
           transferLeft.setPower(1);
           transferRight.setPower(1);
       }
       );
    }

    public Command TransferOut(){
        return new InstantCommand( () -> {
            transferLeft.setPower(-1);
            transferRight.setPower(-1);
        }
        );
    }

}
