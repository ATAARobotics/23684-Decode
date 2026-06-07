package org.firstinspires.ftc.teamcode.Utils;


import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.PedroPathing.Constants;

public class TeleOpDrive {
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor ;

    public TeleOpDrive(HardwareMap hardwareMap){
        frontLeftMotor = hardwareMap.dcMotor.get(Constants.driveConstants.leftFrontMotorName);
        backLeftMotor = hardwareMap.dcMotor.get(Constants.driveConstants.leftRearMotorName);
        frontRightMotor = hardwareMap.dcMotor.get(Constants.driveConstants.rightFrontMotorName);
        backRightMotor = hardwareMap.dcMotor.get(Constants.driveConstants.rightRearMotorName);

        frontLeftMotor.setDirection(Constants.driveConstants.leftFrontMotorDirection);
        backLeftMotor.setDirection(Constants.driveConstants.leftRearMotorDirection);
        frontRightMotor.setDirection(Constants.driveConstants.rightFrontMotorDirection);
        backRightMotor.setDirection(Constants.driveConstants.rightRearMotorDirection);
    }
    public void TeleopDrive(Follower follower, double xDir, double yDir, double hDir) {

            double y = -yDir; // Remember, Y stick value is reversed
            double x = xDir;
            double rx = hDir;


            double botHeading = follower.getPose().getHeading();

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

    }
}