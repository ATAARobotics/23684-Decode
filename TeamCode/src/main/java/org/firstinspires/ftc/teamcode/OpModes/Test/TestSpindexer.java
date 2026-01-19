package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.Subsystem.Spindexer;

public class TestSpindexer extends OpMode {
    private Spindexer spindexer;
    CommandScheduler scheduler;

    boolean spindexerPressed;
    @Override
    public void init() {
        spindexer = new Spindexer(hardwareMap);
        scheduler = CommandScheduler.getInstance();
    }

    @Override
    public void loop() {

        if (gamepad1.a && !spindexerPressed) {
            scheduler.schedule(spindexer.NextTarget());
            spindexerPressed = true;
        }else if (!gamepad1.a && spindexerPressed) {
            spindexerPressed = false;
        }


    }
}
