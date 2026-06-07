package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

public class BeamBreaker extends SubsystemBase{
    DigitalChannel intakeBeamBreaker;

    public BeamBreaker(HardwareMap hardwareMap) {
        intakeBeamBreaker = hardwareMap.get(DigitalChannel.class, "intakeBeamBreaker");
        intakeBeamBreaker.setMode(DigitalChannel.Mode.OUTPUT);
    }
    public boolean isBeamBroken(){
        return intakeBeamBreaker.getState();
    }

    public Command WaitForBeamCut(){
        return new WaitUntilCommand(()->isBeamBroken());
    }

}
