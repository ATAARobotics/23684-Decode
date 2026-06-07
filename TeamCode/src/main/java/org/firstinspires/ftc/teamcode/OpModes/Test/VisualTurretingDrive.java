package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.Limelight;
import org.firstinspires.ftc.teamcode.Utils.TeleOpDrive;


@Configurable
@TeleOp(name = "Visual Turreting Drive", group = "Test")
public class VisualTurretingDrive extends OpMode {

    boolean headingLock = true;
    double currentHeading;
    double targetHeading = Math.toRadians(180);
    double headingCorrection;

    PIDFController headingPIDController;
    TeleOpDrive teleOpDrive;

    public static double P = 0.03 ,I,D,F = 0.001;

    Follower follower;

    Limelight limelight;

    public static double calculateShotAngle(double x, double y, double goalX, double goalY) {
        double deltaX = goalX - x;
        double deltaY = goalY - y;
        return Math.atan2(-deltaY, -deltaX);
    }

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(63.450, 9, Math.toRadians(270)));

        headingPIDController = new PIDFController(new PIDFCoefficients(P,I,D,F));

        teleOpDrive = new TeleOpDrive(hardwareMap);

        limelight = new Limelight(hardwareMap);

    }
    @Override
    public void start() {
        limelight.start();
    }


    @Override
    public void loop() {
        follower.update();
        if (limelight.blueGoalFound()){
            currentHeading = limelight.AngleFrom(Limelight.BLUEGOAL);
            targetHeading = 0;
            headingPIDController.setCoefficients(new PIDFCoefficients(P,I,D,F));
            headingPIDController.updatePosition(-currentHeading);


        }else{
            currentHeading = follower.getPose().getHeading();
            targetHeading = calculateShotAngle(follower.getPose().getX(),follower.getPose().getY(),0,144);
            headingPIDController.setCoefficients(Constants.followerConstants.coefficientsHeadingPIDF);
            headingPIDController.updatePosition(-currentHeading);
        }



        headingLock = gamepad1.right_trigger > 0;

            if(headingLock) {


                double headingError = targetHeading - currentHeading;
                headingError = Math.IEEEremainder(headingError, 2 * Math.PI);

                if (Math.abs(headingError) < Math.toRadians(7)) {
                    headingCorrection = 0;
                } else {
                    headingPIDController.setTargetPosition(targetHeading);
                    headingCorrection = headingPIDController.run();
                }

                teleOpDrive.TeleopDrive(follower,gamepad1.left_stick_x,gamepad1.left_stick_y,headingCorrection);


            } else {
               teleOpDrive.TeleopDrive(follower,gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x);
            }

            telemetry.addLine(follower.getPose().toString());

            limelight.Telemetry(telemetry);
            telemetry.addData("target",targetHeading);
            telemetry.addData("error",Math.abs( targetHeading - currentHeading));
            telemetry.update();

    }
}
