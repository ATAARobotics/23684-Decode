package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


@Config
@TeleOp
public class CoachPrattFlywheel extends OpMode {
    DcMotorEx upperShooter;
    DcMotorEx lowerShooter;

     FtcDashboard dashboard;


     double highVelocity = 1500;
     double lowVelocity = 500;
     double currentVelocity = highVelocity;

    public static double uP = 0, uF = 0;
    public static double lP = 0, lF = 0;

    double[] stepSizes = {10, 1,0.1,0.01};
    int stepIndex = 0;




    @Override
    public void init() {

        upperShooter = hardwareMap.get(DcMotorEx.class, "upperShooter");
        lowerShooter = hardwareMap.get(DcMotorEx.class, "lowerShooter");
        dashboard = FtcDashboard.getInstance();

        upperShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients upperPIDF = new PIDFCoefficients(uP, 0, 0, uF);
        PIDFCoefficients lowerPIDF = new PIDFCoefficients(lP, 0, 0, lF);
        upperShooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, upperPIDF);
        lowerShooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, lowerPIDF);

        telemetry.addLine("I did the bare minimum :D");


    }

    @Override
    public void loop() {

        if (gamepad1.yWasPressed()) {
            if (currentVelocity == highVelocity) {
                currentVelocity = lowVelocity;
            } else {
                currentVelocity = highVelocity;
            }

        }
         if (gamepad1.bWasPressed()) {
             stepIndex = (stepIndex + 1) % stepSizes.length;
         }

         if (gamepad1.dpadLeftWasPressed()){
             uF -= stepSizes[stepIndex];
         }

         if (gamepad1.dpadRightWasPressed()){
             uF += stepSizes[stepIndex];
         }

         if (gamepad1.dpadUpWasPressed()){
             uP += stepSizes[stepIndex];
         }
         if (gamepad1.dpadDownWasPressed()){
             uP -= stepSizes[stepIndex];
         }

         PIDFCoefficients pidf = new PIDFCoefficients(uP, 0, 0, uF);
         upperShooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);

         upperShooter.setPower(currentVelocity);

         double currentVello = upperShooter.getVelocity();
         double error = currentVelocity - currentVello;
         telemetry.addData("Current Velocity", currentVello);
         dashboard.getTelemetry().addData("Current Velocity", currentVello);
        dashboard.getTelemetry().addData("Error", currentVello);
        dashboard.getTelemetry().addData("Target Velocity", currentVelocity);
         telemetry.addData("Error", error);
         telemetry.update();







    }
}
