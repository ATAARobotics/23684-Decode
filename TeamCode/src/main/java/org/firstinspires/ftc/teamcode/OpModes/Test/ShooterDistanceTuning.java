package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.Subsystem.Limelight;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;
public class ShooterDistanceTuning extends OpMode {
    Shooter shooter;
    CommandScheduler scheduler;

    Limelight limelight;

    double upperMotorTarget = 0;

    @Override
    public void init() {
        scheduler = CommandScheduler.getInstance();
        shooter = new Shooter(hardwareMap);
        limelight = new Limelight(hardwareMap);
    }

    @Override
    public void loop() {
        limelight.update();


    }
}
