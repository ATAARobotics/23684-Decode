package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Utills.ShootAngle;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class holdPoseDrive extends OpMode {

    public double X = 0;
    public double Y = 0;
    public double heading = 0;

    public double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    Follower follower;
    Pose startPose = new Pose(72,72,270);
    Pose joystickToDeltaPose() {
        double MAX_TRANSLATION_VEL = 12;
        double MAX_ANGULAR_VEL = Math.PI / 4;

        double xInput = -gamepad1.left_stick_y;
        double yInput = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        // Deadband
        if (Math.abs(xInput) < 0.05) xInput = 0;
        if (Math.abs(yInput) < 0.05) yInput = 0;
        if (Math.abs(turn) < 0.05) turn = 0;

        // Scale to velocities
        double vx = xInput * MAX_TRANSLATION_VEL;
        double vy = yInput * MAX_TRANSLATION_VEL;
        double vHeading = turn * MAX_ANGULAR_VEL;

        X += vx;
        Y += vy;
        if (gamepad1.left_trigger < 0.9) {
            heading += vHeading;
            heading = normalizeAngle(heading);
        }else{
            heading = ShootAngle.calculateShotAngle(follower.getPose().getX(), follower.getPose().getY(), 0, 144);
        }

        return new Pose(X,Y,heading);

    }
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
    }

    @Override
    public void loop() {
        follower.update();
        follower.holdPoint(joystickToDeltaPose());

    }
}
