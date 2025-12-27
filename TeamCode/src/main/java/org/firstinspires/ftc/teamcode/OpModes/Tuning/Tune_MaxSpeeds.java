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

		// Drive forward at full power for 1.5 seconds or until user stops
		while (opModeIsActive() && timer.seconds() < 1.5) {
			// Command full forward power
			drive.setDrivePowers(new PoseVelocity2d(new Vector2d(1.0, 0.0), 0.0));

			// Update localization and get current velocity
			PoseVelocity2d currentPoseVel = drive.updatePoseEstimate();

			// Calculate Linear Velocity (hypotenuse of x and y, though y should be 0)
			double currentVel = Math.hypot(currentPoseVel.linearVel.x, currentPoseVel.linearVel.y);
			double currentTime = timer.seconds();

			// Calculate Acceleration (dV / dt)
			// We filter small time steps to avoid noise spikes
			if (currentTime - previousTime > 0.01) {
				double currentAccel = (currentVel - previousVel) / (currentTime - previousTime);

				// Capture peaks
				if (currentVel > maxVel) maxVel = currentVel;
				if (currentAccel > maxAccel) maxAccel = currentAccel;

				previousVel = currentVel;
				previousTime = currentTime;
			}
		}

		// Stop
		drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
		sleep(1500); // Wait for robot to settle

		// ----------------------------------------------------------------
		// TEST 2: ANGULAR VELOCITY
		// ----------------------------------------------------------------
		telemetry.addLine("Running Spin Test...");
		telemetry.update();

		double maxAngVel = 0.0;
		timer.reset();

		while (opModeIsActive() && timer.seconds() < 1.5) {
			// Command full spin power
			drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), 1.0));

			PoseVelocity2d currentPoseVel = drive.updatePoseEstimate();
			double currentAngVel = Math.abs(currentPoseVel.angVel);

			if (currentAngVel > maxAngVel) maxAngVel = currentAngVel;
		}

		// Stop
		drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

		// ----------------------------------------------------------------
		// REPORT RESULTS
		// ----------------------------------------------------------------
		// Apply 80% safety factor for reliable path following
		double recMaxVel = maxVel * 0.85;
		double recMaxAccel = maxAccel * 0.80; // Acceleration needs more headroom to prevent slip
		double recMaxAngVel = maxAngVel * 0.85;

		// Display results forever
		while (opModeIsActive()) {
			telemetry.addLine("=== TUNING COMPLETE ===");

			telemetry.addData("Measured Max Vel", "%.2f", maxVel);
			telemetry.addData("Measured Max Accel", "%.2f", maxAccel);
			telemetry.addData("Measured Max AngVel", "%.2f", maxAngVel);

			telemetry.addLine("\n=== RECOMMENDED CONFIG ===");
			telemetry.addLine("Update these in MecanumDrive.java Params:");
			telemetry.addData("maxProfileVel", "%.2f", recMaxVel);
			telemetry.addData("maxProfileAccel", "%.2f", recMaxAccel);
			telemetry.addData("maxAngVel", "%.2f", recMaxAngVel);

			telemetry.update();
		}
	}
}

