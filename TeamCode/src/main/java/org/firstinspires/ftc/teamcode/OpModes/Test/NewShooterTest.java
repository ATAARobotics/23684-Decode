package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


@Configurable
@TeleOp
public class NewShooterTest extends OpMode {

    DcMotorEx upperShooter;
    DcMotorEx lowerShooter;



    public static double UpperP = 15, UpperF = 15.19;
    public static double LowerP = 15 ,LowerF = 15.4;



    double lowVelocity = 900;
    double highVelocity = 1500;

    double curTargetVelocity = 0;

    double upperRPM;
    double lowerRPM;

    double TICKS_PER_REVOLUTION = 28.0;

    double RPM_CONVERSION = 60.0 / TICKS_PER_REVOLUTION;

    double TPR_CONVERSION(double RPM){
        return (RPM * TICKS_PER_REVOLUTION) / 60.0;
    }



    private boolean upper;

    public static double targetUpper, targetLower;

    double[] stepsizes = {10.0, 1.0, 0.1, 0.01, 0.001};

    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();



    int stepindex = 1;
    @Override
    public void init() {
        upperShooter = hardwareMap.get(DcMotorEx.class, "upperShooter");
        lowerShooter = hardwareMap.get(DcMotorEx.class, "lowerShooter");

        PIDFCoefficients pidfCoefficientsUpper = new PIDFCoefficients(UpperP, 0, 0, UpperF);
        PIDFCoefficients pidfCoefficientsLower = new PIDFCoefficients(LowerP, 0, 0, LowerF);

        upperShooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficientsUpper);
        lowerShooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficientsLower);


    }

    @Override
    public void loop() {

//        if (gamepad1.yWasPressed()){
//           if(curTargetVelocity == highVelocity) curTargetVelocity = lowVelocity;
//           else curTargetVelocity = highVelocity;
//        }
//        else if (gamepad1.bWasPressed()){
//            stepindex = (stepindex + 1) % stepsizes.length;
//        }
////
//        if(gamepad1.dpadUpWasPressed()){
//            if(upper){
//                UpperF += stepsizes[stepindex];
//            }else LowerF += stepsizes[stepindex];
//
//        }

//        if(gamepad1.dpadDownWasPressed()){
//            if(upper){
//                UpperF -= stepsizes[stepindex];
//            }else LowerF -= stepsizes[stepindex];
//        }
//
//        if(gamepad1.dpadLeftWasPressed()){
//            if(upper){
//                UpperP -= stepsizes[stepindex];
//            }else LowerP -= stepsizes[stepindex];
//        }
//
//        if(gamepad1.dpadRightWasPressed()){
//            if(upper){
//                UpperP += stepsizes[stepindex];
//            }else LowerP += stepsizes[stepindex];
//        }

//        if (gamepad1.rightTriggerWasPressed()) upper = true;
//        else upper = false;

        PIDFCoefficients pidfCoefficientsUpper = new PIDFCoefficients(UpperP, 0, 0, UpperF);
        PIDFCoefficients pidfCoefficientsLower = new PIDFCoefficients(LowerP, 0, 0, LowerF);

        upperShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficientsUpper);
        lowerShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficientsLower);

        upperShooter.setVelocity(targetUpper);
        lowerShooter.setVelocity(targetLower);

        upperRPM = upperShooter.getVelocity() * RPM_CONVERSION;
        lowerRPM = lowerShooter.getVelocity() * RPM_CONVERSION;


//        telemetry.addLine("curentMotor " + (upper ? "Uppermotor" : "Lower"));
//        telemetry.addLine("===F===");
//        telemetry.addData("UpperF", UpperF);
//        telemetry.addData("LowerF", LowerF);
//        telemetry.addLine("===F===");
//        telemetry.addData("UpperP", UpperP);
//        telemetry.addData("LowerP", LowerP);
//
//        telemetry.addLine("  ");
//        telemetry.addData("error", (upper ?  curTargetVelocity - upperShooter.getVelocity()  : curTargetVelocity - lowerShooter.getVelocity() ) );
//        telemetry.update();


//        panelsTelemetry.addLine("curentMotor " + (upper ? "Uppermotor" : "Lower"));
//        panelsTelemetry.addLine("===F===");
//        panelsTelemetry.addData("UpperF", UpperF);
//        panelsTelemetry.addData("LowerF", LowerF);
//        panelsTelemetry.addLine("===F===");
//        panelsTelemetry.addData("UpperP", UpperP);
//        panelsTelemetry.addData("LowerP", LowerP);

        panelsTelemetry.addData("upper rpm",upperShooter.getVelocity());
        panelsTelemetry.addData("lower rpm",lowerShooter.getVelocity());


        panelsTelemetry.addData("upper target",targetUpper);

        panelsTelemetry.addData("lowertarget",targetLower);

        panelsTelemetry.addLine("  ");
        panelsTelemetry.addData("error", (upper ?  curTargetVelocity - upperShooter.getVelocity()  : curTargetVelocity - lowerShooter.getVelocity() ) );
        panelsTelemetry.update();




    }
}
