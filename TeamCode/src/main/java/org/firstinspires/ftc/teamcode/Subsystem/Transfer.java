package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;

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
        return new CommandBase() {
            @Override
            public void execute() {
                intakeDoorLeft.setPower(1);
                intakeDoorRight.setPower(1);
            }
            @Override
            public boolean isFinished() {
                return true;
            }
        };
    }

    public Command IntakeDoorOut(){
        return new CommandBase() {
            @Override
            public void execute() {
                intakeDoorLeft.setPower(-1);
                intakeDoorRight.setPower(-1);
            }
            @Override
            public boolean isFinished() {
                return true;
            }
        };
    }

   public Command TransferIn(){
        return new CommandBase() {
            @Override
            public void execute() {
                transferLeft.setPower(1);
                transferRight.setPower(1);
            }
            @Override
            public boolean isFinished() {
                return true;
            }
        };
    }

    public Command TransferOut(){
        return new CommandBase() {
            @Override
            public void execute() {
                transferLeft.setPower(-1);
                transferRight.setPower(-1);
            }
            @Override
            public boolean isFinished() {
                return true;
            }
        };
    }

}
