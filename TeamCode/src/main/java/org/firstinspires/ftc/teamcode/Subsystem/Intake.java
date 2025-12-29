package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.Subsystem;

import java.util.Collections;
import java.util.Set;

public class Intake {
    DcMotor intake;

    double INSPEED = -0.5;
    double OUTSPEED = 0.5;
    double SLOWSPEED = -0.2;

    public Intake(HardwareMap hardwareMap){
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    Command In(){
        return new CommandBase() {
            @Override
            public void execute() {
                intake.setPower(INSPEED);
            }
            @Override
            public boolean isFinished() {
                return intake.isBusy() && intake.getPower() == INSPEED;
            }
        };
    }

    Command Slow(){
        return new CommandBase() {
            @Override
            public void execute() {
                intake.setPower(SLOWSPEED);
            }
            @Override
            public boolean isFinished() {
                return intake.isBusy() && intake.getPower() == SLOWSPEED;
            }
        };
    }

    Command Out(){
        return new CommandBase() {
            @Override
            public void execute() {
                intake.setPower(OUTSPEED);
            }
            @Override
            public boolean isFinished() {
                return intake.isBusy() && intake.getPower() == OUTSPEED;
            }
        };
    }

    Command Stop(){
        return new CommandBase() {
            @Override
            public void execute() {
                intake.setPower(0);
            }
            @Override
            public boolean isFinished() {
                return true;
            }
        };
    }
}
