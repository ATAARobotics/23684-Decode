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
import org.firstinspires.ftc.teamcode.Utils.RobotConfig;
import org.firstinspires.ftc.teamcode.Utils.Team;
import org.firstinspires.ftc.teamcode.Utils.Drive;


@Configurable
@TeleOp(name = "Visual Turreting Drive", group = "Test")
public class VisualTurretingDrive extends OpMode {

	boolean headingLock = true;
	double currentHeading;
	double targetHeading = Math.toRadians(180);
	double headingCorrection;

	PIDFController headingPIDController;
	Drive drive;

	public static double P = 0.025, I, D = 0.00025, F = 0.001;

	double headingdeadzone =0;

	Follower follower;

	Limelight limelight;

	public double calculateShotAngle(double x, double y, double goalX, double goalY) {
		double deltaX = goalX - x;
		double deltaY = goalY - y;
		return Math.atan2(-deltaY, -deltaX);
	}

	@Override
	public void init() {

		follower = Constants.createFollower(hardwareMap);
		follower.setStartingPose(new Pose(63.450, 9, Math.toRadians(270)));

		headingPIDController = new PIDFController(new PIDFCoefficients(P, I, D, F));

		drive = new Drive(hardwareMap);

		limelight = new Limelight(hardwareMap);

	}

	@Override
	public void start() {
		limelight.start();
	}


	@Override
	public void loop() {
		follower.update();
		if (limelight.goalsFound(Team.BLUE)) {
			currentHeading = limelight.AngleFrom(Team.BLUE);
			targetHeading = 0;
			headingPIDController.setCoefficients(new PIDFCoefficients(P, I, D, F));
			headingPIDController.updatePosition(-currentHeading);
			headingdeadzone = 1;


		} else {
			currentHeading = follower.getHeading();
			targetHeading = limelight.calculateShotAngle(follower.getPose().getX(), follower.getPose().getY(), 0, 144);
			headingPIDController.setCoefficients(Constants.followerConstants.coefficientsHeadingPIDF);
			headingPIDController.updatePosition(-currentHeading);
			headingdeadzone = 3;
		}


		headingLock = gamepad1.right_trigger > 0;

		if (headingLock) {


			double headingError = targetHeading - currentHeading;
			headingError = Math.IEEEremainder(headingError, 2 * Math.PI);

			if (Math.abs(headingError) < Math.toRadians(headingdeadzone)) {
				headingCorrection = 0;
			} else {
				headingPIDController.setTargetPosition(targetHeading);
				headingCorrection = headingPIDController.run();
			}

			drive.TeleopDrive(follower, gamepad1.left_stick_x, gamepad1.left_stick_y, headingCorrection);


		} else {
			drive.TeleopDrive(follower, gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
		}

		if (RobotConfig.COMPETITION) return;

		telemetry.addLine(follower.getPose().toString());

		limelight.Telemetry(telemetry);
		telemetry.addData("heading",follower.getHeading());
		telemetry.addData("target", Math.toDegrees(targetHeading));
		telemetry.addData("error", Math.abs(targetHeading - currentHeading));
		telemetry.update();

	}
}
