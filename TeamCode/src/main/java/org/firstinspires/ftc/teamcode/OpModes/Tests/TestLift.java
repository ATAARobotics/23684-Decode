package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TestLift extends OpMode {

    DcMotor decodeTest;
    @Override
    public void init() {
       decodeTest = hardwareMap.get(DcMotor.class, "decodeTest");
    }

    @Override
    public void loop() {
        decodeTest.setPower(gamepad1.left_stick_y / 2);

    }
}
