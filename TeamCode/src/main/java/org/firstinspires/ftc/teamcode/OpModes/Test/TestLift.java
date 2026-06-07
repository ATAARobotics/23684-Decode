package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test Lift (TEST BENCH)", group = "Test")
public class TestLift extends OpMode {
    DcMotor motorA;
    DcMotor motorB;

    Servo linearServo;


    @Override
    public void init() {
        motorA = hardwareMap.dcMotor.get("motorA");
        motorB = hardwareMap.dcMotor.get("motorB");
        linearServo = hardwareMap.get(Servo.class, "linearServo");

        motorB.setDirection(DcMotor.Direction.REVERSE);



    }
    @Override
    public void start(){
        linearServo.setPosition(0);
    }

    @Override
    public void loop() {
        motorA.setPower(gamepad1.left_stick_y);
        motorB.setPower(gamepad1.left_stick_y);

        if(gamepad1.a){
            linearServo.setPosition(0.4);
        }
        if(gamepad1.b){
            linearServo.setPosition(0.5);
        }


    }
}
