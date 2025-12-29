package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


@Autonomous
public class StateMachineTestOpMode extends OpMode {

    StateMachineTest statey;
    @Override
    public void init() {
        statey = new StateMachineTest();
        statey.init(hardwareMap,telemetry,gamepad1);

    }

    @Override
    public void loop() {
        statey.updateStateMachine();

    }
}
