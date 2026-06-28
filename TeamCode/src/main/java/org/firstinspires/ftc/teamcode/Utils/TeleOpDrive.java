package org.firstinspires.ftc.teamcode.Utils;


import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.PedroPathing.Constants;

public class TeleOpDrive {
	DcMotor frontLeftMotor;
	DcMotor backLeftMotor;
	DcMotor frontRightMotor;
	DcMotor backRightMotor;

	public TeleOpDrive(HardwareMap hardwareMap) {
		frontLeftMotor = hardwareMap.dcMotor.get(Constants.driveConstants.leftFrontMotorName);
		backLeftMotor = hardwareMap.dcMotor.get(Constants.driveConstants.leftRearMotorName);
		frontRightMotor = hardwareMap.dcMotor.get(Constants.driveConstants.rightFrontMotorName);
		backRightMotor = hardwareMap.dcMotor.get(Constants.driveConstants.rightRearMotorName);

		frontLeftMotor.setDirection(Constants.driveConstants.leftFrontMotorDirection);
		backLeftMotor.setDirection(Constants.driveConstants.leftRearMotorDirection);
		frontRightMotor.setDirection(Constants.driveConstants.rightFrontMotorDirection);
		backRightMotor.setDirection(Constants.driveConstants.rightRearMotorDirection);

		frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	}

	public void TeleopDrive(Follower follower, double xDir, double yDir, double hDir) {

		double y = -yDir; // Remember, Y stick value is reversed
		double x = xDir;
		double rx = hDir;


		// Rotate the movement direction counter to the bot's rotation
		double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
		double frontLeftPower = (y + x + rx) / denominator;
		double backLeftPower = (y - x + rx) / denominator;
		double frontRightPower = (y - x - rx) / denominator;
		double backRightPower = (y + x - rx) / denominator;

		frontLeftMotor.setPower(frontLeftPower);
		backLeftMotor.setPower(backLeftPower);
		frontRightMotor.setPower(frontRightPower);
		backRightMotor.setPower(backRightPower);

	}
}