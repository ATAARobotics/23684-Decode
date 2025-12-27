package org.firstinspires.ftc.teamcode.OpModes.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;

@TeleOp(group = "Tuning")
public class Tune_MaxSpeeds extends LinearOpMode {

	@Override
	public void runOpMode() throws InterruptedException {
		MecanumDrive drive = new MecanumDrive(hardwareMap, new com.acmerobotics.roadrunner.Pose2d(0, 0, 0));
		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

		telemetry.addLine("WARNING: Robot will move FAST!");
		telemetry.addLine("Ensure 8ft of clear space ahead.");
		telemetry.addLine("Press START to begin.");
		telemetry.update();

		waitForStart();

		// ----------------------------------------------------------------
		// TEST 1: FORWARD VELOCITY & ACCELERATION
		// ----------------------------------------------------------------
		telemetry.addLine("Running Forward Test...");
		telemetry.update();

		double maxVel = 0.0;
		double maxAccel = 0.0;
		double previousVel = 0.0;
		double previousTime = 0.0;

		ElapsedTime timer = new ElapsedTime();

		// Drive forward at full power
		// Increased timing to ensure max velocity is reached
		while (opModeIsActive() && timer.seconds() < 2.0) {
			drive.setDrivePowers(new PoseVelocity2d(new Vector2d(1.0, 0.0), 0.0));

			PoseVelocity2d currentPoseVel = drive.updatePoseEstimate();
			double currentVel = Math.hypot(currentPoseVel.linearVel.x, currentPoseVel.linearVel.y);
			double currentTime = timer.seconds();

			// Calculate Linear Acceleration (dV / dt)
			if (currentTime - previousTime > 0.01) { // Filter small time steps
				double currentAccel = Math.abs(currentVel - previousVel) / (currentTime - previousTime);

				if (currentVel > maxVel) maxVel = currentVel;
				if (currentAccel > maxAccel) maxAccel = currentAccel;

				previousVel = currentVel;
				previousTime = currentTime;
			}
		}

		// Stop and Settle
		drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
		sleep(2000);

		// ----------------------------------------------------------------
		// TEST 2: ANGULAR VELOCITY & ACCELERATION
		// ----------------------------------------------------------------
		telemetry.addLine("Running Spin Test...");
		telemetry.update();

		double maxAngVel = 0.0;
		double maxAngAccel = 0.0;
		double previousAngVel = 0.0;
		previousTime = 0.0; // Reset time tracker logic

		timer.reset();

		// Spin at full power
		while (opModeIsActive() && timer.seconds() < 4.0) {
			drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), 1.0));

			PoseVelocity2d currentPoseVel = drive.updatePoseEstimate();
			double currentAngVel = Math.abs(currentPoseVel.angVel);
			double currentTime = timer.seconds();

			// Calculate Angular Acceleration (dOmega / dt)
			if (currentTime - previousTime > 0.01) {
				double currentAngAccel = Math.abs(currentAngVel - previousAngVel) / (currentTime - previousTime);

				if (currentAngVel > maxAngVel) maxAngVel = currentAngVel;
				if (currentAngAccel > maxAngAccel) maxAngAccel = currentAngAccel;

				previousAngVel = currentAngVel;
				previousTime = currentTime;
			}
		}

		// Stop
		drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

		// ----------------------------------------------------------------
		// REPORT RESULTS
		// ----------------------------------------------------------------
		// Apply safety margins (85% for vel, 80% for accel to prevent slip)
		double recMaxVel = maxVel * 0.85;
		double recMaxAccel = maxAccel * 0.80;
		double recMaxAngVel = maxAngVel * 0.85;
		double recMaxAngAccel = maxAngAccel * 0.80;

		while (opModeIsActive()) {
			telemetry.addLine("=== TUNING COMPLETE ===");

			telemetry.addData("Measured Max Vel", "%.2f", maxVel);
			telemetry.addData("Measured Max Accel", "%.2f", maxAccel);
			telemetry.addData("Measured Max AngVel", "%.2f", maxAngVel);
			telemetry.addData("Measured Max AngAccel", "%.2f", maxAngAccel);

			telemetry.addLine("\n=== RECOMMENDED CONFIG ===");
			telemetry.addLine("Update in MecanumDrive.java Params:");
			telemetry.addData("maxProfileVel", "%.2f", recMaxVel);
			telemetry.addData("maxProfileAccel", "%.2f", recMaxAccel);
			telemetry.addData("maxAngVel", "%.2f", recMaxAngVel);
			telemetry.addData("maxAngAccel", "%.2f", recMaxAngAccel);

			telemetry.update();
		}
	}
}
