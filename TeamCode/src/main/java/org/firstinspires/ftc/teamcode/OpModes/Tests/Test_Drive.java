package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utilities.PIDFController;
import org.firstinspires.ftc.teamcode.Utilities.Angle;

/**
 * Drive test OpMode for testing Mecanum drive with PID control.
 * Uses GoBilda Pinpoint for odometry.
 */
@Config
@TeleOp
public class Test_Drive extends OpMode {
	DcMotor frontRight;
	DcMotor rearRight;
	DcMotor frontLeft;
	DcMotor rearLeft;

	// Tuning parameters
	public static double targetX = 0;
	public static double targetY = 0;
	public static double targetH = 0;
	public double prevHeading = 0;
	public double prevUnwrappedHeading = 0;

	// Drive system components
	private Pose2d pose;
	private Pose2d target;
	private GoBildaPinpointDriver pinpoint;
	private PIDFController xPidController;
	private PIDFController yPidController;
	private PIDFController headingPidController;

	private MecanumDrive drive;
	private static final double DEADZONE_THRESHOLD = 0.05;

	@Override
	public void init() {
		pose = new Pose2d(0, 0, 0);
		target = new Pose2d(targetX, targetY, targetH);

		xPidController = new PIDFController(Xcontroller.kP, Xcontroller.kI, Xcontroller.kD, Xcontroller.kF);
		yPidController = new PIDFController(Ycontroller.kP, Ycontroller.kI, Ycontroller.kD, Ycontroller.kF);
		headingPidController = new PIDFController(Hcontroller.kP, Hcontroller.kI, Hcontroller.kD, Hcontroller.kF);

		GoBildaPinpointDriver.EncoderDirection yEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
		GoBildaPinpointDriver.EncoderDirection xEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;

		pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
		pinpoint.setOffsets(-177.8, -63.5, DistanceUnit.MM);
		pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
		pinpoint.setEncoderDirections(xEncoderDirection, yEncoderDirection);
		pinpoint.resetPosAndIMU();

		drive = new MecanumDrive(hardwareMap,pose);
		frontRight = hardwareMap.get(DcMotor.class, "frontRight");
		rearRight = hardwareMap.get(DcMotor.class, "rearRight");
		frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
		rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");

		prevHeading = pinpoint.getHeading(AngleUnit.DEGREES);

	}

	@Override
	public void loop() {
		// Update PID gains from config
		xPidController.setPID(Xcontroller.kP, Xcontroller.kI, Xcontroller.kD, Xcontroller.kF);
		yPidController.setPID(Ycontroller.kP, Ycontroller.kI, Ycontroller.kD, Ycontroller.kF);
		headingPidController.setPID(Hcontroller.kP, Hcontroller.kI, Hcontroller.kD, Hcontroller.kF);

		// Update odometry
		pinpoint.update();
		pose = new Pose2d(
				-pinpoint.getPosY(DistanceUnit.INCH),
				pinpoint.getPosX(DistanceUnit.INCH),
				pinpoint.getHeading(AngleUnit.RADIANS)
		);
		target = new Pose2d(targetX, targetY, Math.toRadians(targetH));

		prevUnwrappedHeading = Angle.unwrap(prevHeading, Math.toDegrees(pose.heading.toDouble()), prevUnwrappedHeading, 180);

		// Calculate power commands
		double forwardPower = yPidController.getOutput(pose.position.y, target.position.y);
		double strafePower = xPidController.getOutput(pose.position.x, target.position.x);
		double turnPower = headingPidController.getOutput(
				prevUnwrappedHeading,
				Angle.nearestCoterminal(Math.toDegrees(target.heading.toDouble()), prevUnwrappedHeading, 180)
		);
		prevHeading = Math.toDegrees(pose.heading.toDouble());

		// Apply deadzone
		forwardPower = Math.abs(forwardPower) > DEADZONE_THRESHOLD ? forwardPower : 0;
		strafePower = Math.abs(strafePower) > DEADZONE_THRESHOLD ? strafePower : 0;
		turnPower = Math.abs(turnPower) > DEADZONE_THRESHOLD ? turnPower : 0;


		double rotX = strafePower * Math.cos(-pose.heading.toDouble()) - forwardPower * Math.sin(-pose.heading.toDouble());
		double rotY = strafePower * Math.sin(-pose.heading.toDouble()) + forwardPower * Math.cos(-pose.heading.toDouble());

		rotX = rotX * 1.1;  // Counteract imperfect strafing

		// Denominator is the largest motor power (absolute value) or 1
		// This ensures all the powers maintain the same ratio,
		// but only if at least one is out of the range [-1, 1]
		double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turnPower), 1);
		double frontLeftPower = (rotY + rotX + turnPower) / denominator;
		double backLeftPower = (rotY - rotX + turnPower) / denominator;
		double frontRightPower = (rotY - rotX - turnPower) / denominator;
		double backRightPower = (rotY + rotX - turnPower) / denominator;

		frontLeft.setPower(frontLeftPower);
		rearLeft.setPower(backLeftPower);
		frontRight.setPower(frontRightPower);
		rearRight.setPower(backRightPower);

		// Telemetry
		telemetry.addData("Position", pose.position);
		telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading.toDouble()));
		telemetry.addData("Heading Target",Math.toDegrees(target.heading.toDouble()));
		telemetry.addLine();
		telemetry.addData("Forward Power", forwardPower);
		telemetry.addData("Strafe Power", strafePower);
		telemetry.addData("Turn Power", turnPower);
		telemetry.update();
	}

	/**
	 * X-axis (strafe) PID controller gains.
	 */
	public static class XController {
		public static double kP = 0.1;
		public static double kI = 0;
		public static double kD = 0.01;
		public static double kF = 0;
	}

	public static XController Xcontroller = new XController();

	/**
	 * Y-axis (forward) PID controller gains.
	 */
	public static class YController {
		public static double kP = 0.1;
		public static double kI = 0;
		public static double kD = 0.01;
		public static double kF = 0;
	}

	public static YController Ycontroller = new YController();

	/**
	 * Heading (rotation) PID controller gains.
	 */
	public static class HController {
		public static double kP = 0;
		public static double kI = 0;
		public static double kD = 0;
		public static double kF = 0;
	}

	public static HController Hcontroller = new HController();
}
