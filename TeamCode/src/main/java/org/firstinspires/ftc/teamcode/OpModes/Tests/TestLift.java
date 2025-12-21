package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TestLift extends OpMode {

    DcMotor decodeTest;
    CRServo servo;

    @Override
    public void init() {
       decodeTest = hardwareMap.get(DcMotor.class, "decodeTest");
       servo = hardwareMap.get(CRServo.class, "billy");
    }

    @Override
    public void loop() {
        decodeTest.setPower(gamepad1.left_stick_y );
        servo.setPower(gamepad1.right_stick_y);

    }
}
