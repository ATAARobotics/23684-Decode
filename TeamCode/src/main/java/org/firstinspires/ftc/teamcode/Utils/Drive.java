package org.firstinspires.ftc.teamcode.Utils;


import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.PedroPathing.Constants;

public class Drive {
	DcMotorEx frontLeftMotor;
	DcMotorEx backLeftMotor;
	DcMotorEx frontRightMotor;
	DcMotorEx backRightMotor;

	protected static double FULL_POWER_VELOCITY = 2400;

	com.pedropathing.control.PIDFController headingPIDController;


	public static PIDFCoefficients coefficientsHeadingPIDF = new PIDFCoefficients(0.03, 0,  0.0003, 0.001);
	public static PIDFCoefficients coefficientsHeadingPIDFSlow = new PIDFCoefficients(0.003, 0,  0.003, 0.001);

	public double calculateShotAngle(double x, double y, double goalX, double goalY) {
		double deltaX = goalX - x;
		double deltaY = goalY - y;

		double Tanang = Math.atan2(deltaY,deltaX);
		double finalang = Tanang + Math.PI;
		return MathFunctions.normalizeAngleSigned(finalang);
		// this works! for now... theres a 98.42% it will break k-days
	}


	public double anglewrap(double degrees){
		while(degrees >180 ) degrees -= 360;
		while (degrees < -180) degrees +=360;
		return degrees;
	}

	public Drive(HardwareMap hardwareMap) {
		frontLeftMotor = hardwareMap.get(DcMotorEx.class, Constants.driveConstants.leftFrontMotorName);
		backLeftMotor = hardwareMap.get(DcMotorEx.class, Constants.driveConstants.leftRearMotorName);
		frontRightMotor = hardwareMap.get(DcMotorEx.class, Constants.driveConstants.rightFrontMotorName);
		backRightMotor = hardwareMap.get(DcMotorEx.class, Constants.driveConstants.rightRearMotorName);

		frontLeftMotor.setDirection(Constants.driveConstants.leftFrontMotorDirection);
		backLeftMotor.setDirection(Constants.driveConstants.leftRearMotorDirection);
		frontRightMotor.setDirection(Constants.driveConstants.rightFrontMotorDirection);
		backRightMotor.setDirection(Constants.driveConstants.rightRearMotorDirection);

		frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		headingPIDController = new PIDFController(coefficientsHeadingPIDF);
	}

	private void Motorspeed(DcMotorEx motor, double power) {
		double speed = power * FULL_POWER_VELOCITY;
		motor.setVelocity(speed);
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

		Motorspeed(frontLeftMotor, frontLeftPower);
		Motorspeed(backLeftMotor, backLeftPower);
		Motorspeed(frontRightMotor, frontRightPower);
		Motorspeed(backRightMotor, backRightPower);


//		frontLeftMotor.setPower(frontLeftPower);
//		backLeftMotor.setPower(backLeftPower);
//		frontRightMotor.setPower(frontRightPower);
//		backRightMotor.setPower(backRightPower);

	}

}