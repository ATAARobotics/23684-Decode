package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
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

   public Command In(){
        return new InstantCommand(
                ()-> intake.setPower(INSPEED)
        );
    }

    public Command Slow(){
        return new InstantCommand(
                ()-> intake.setPower(SLOWSPEED)
        );
    }

    public Command Out(){
        return new InstantCommand(
                ()-> intake.setPower(OUTSPEED)
        );
    }

    public Command Stop(){
        return new InstantCommand(
                ()-> intake.setPower(0)
        );
    }
}
