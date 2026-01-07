package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

public class Spindexer {
    CRServo spindexer;

    DcMotor spindexerEncoder;

    public Spindexer(HardwareMap hardwareMap) {
        spindexer = hardwareMap.get(CRServo.class, "spindexerLeft");
        spindexerEncoder = hardwareMap.get(DcMotor.class, "intake");
    }

    public Command DirectPower(double power){
        return new CommandBase() {
            @Override
            public void execute() {
                spindexer.setPower(power);
            }
            @Override
            public boolean isFinished() {
                return true;
            }
        };
    }

    private Command NextSlot(){
        return new CommandBase() {
            double target = 0;
            @Override
            public void initialize() {
               target = spindexerEncoder.getCurrentPosition() + ((double)8192/3);
            }

            @Override
            public void execute() {
                spindexerEncoder.setPower(1);
            }

            @Override
            public boolean isFinished() {
                return spindexerEncoder.getCurrentPosition() >= target;
            }
        };
    }

    private Command TwoSlots(){
        return new SequentialCommandGroup(
                NextSlot(),
                new WaitCommand(200),
                NextSlot()
        );
    }


}
