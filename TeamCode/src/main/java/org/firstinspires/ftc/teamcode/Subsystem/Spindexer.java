package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

public class Spindexer {
    CRServo spindexer;

    DcMotor spindexerMotor;

    public Spindexer(HardwareMap hardwareMap) {
        spindexer = hardwareMap.get(CRServo.class, "spindexerLeft");
        spindexerMotor = hardwareMap.get(DcMotor.class, "spindexerMotor");
        spindexerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public Command DirectPower(double power){
        return new InstantCommand(
                ()-> spindexerMotor.setPower(power)
        );
    }

    private Command NextSlot(){
        return new CommandBase() {
            double target = 0;
            @Override
            public void initialize() {
               target = spindexerMotor.getCurrentPosition() + ((double)8192/3);
            }

            @Override
            public void execute() {
                spindexerMotor.setPower(1);
            }

            @Override
            public boolean isFinished() {
                return spindexerMotor.getCurrentPosition() >= target;
            }
        };
    }



}
