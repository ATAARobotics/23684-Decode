package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.LifecycleManagementUtilities.HardwareInitializer;
import org.firstinspires.ftc.teamcode.LifecycleManagementUtilities.HardwareShutdown;
import org.firstinspires.ftc.teamcode.LifecycleManagementUtilities.SubsystemUpdater;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.RGBIndicator;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Utilities.ActionScheduler;
import org.firstinspires.ftc.teamcode.Utilities.ShotAngleUtility;
import org.firstinspires.ftc.teamcode.Utilities.SpindexerPositionUtility;
import org.firstinspires.ftc.teamcode.Utilities.Team;
import org.firstinspires.ftc.teamcode.Utilities.TransferUtility;

/**
 * Main TeleOp OpMode for driver control
 * <p>
 * Gamepad 1 (Driver):
 * - Left Stick: Drive forward/backward/strafe
 * - Right Stick: Turn left/right
 * <p>
 * Gamepad 2 (Operator):
 * - A: Intake in
 * - B: Intake out
 * - X: Transfer forward (forced)
 * - Y: Transfer backward (forced)
 * - LB: Transfer forward
 * - RB: Transfer stop
 * - LT: IntakeDoor open
 * - RT: IntakeDoor close
 * - DPad Up: Spindexer spin forward
 * - DPad Down: Spindexer spin backward
 * - Priority: X then RPM-based
 */
public class MainTeleOp extends OpMode {
	protected MecanumDrive drive;
	protected ActionScheduler scheduler;
	protected Shooter shooter;
	protected Intake intake;
	protected Transfer transfer;
	protected Spindexer spindexer;
	protected Limelight limelight;
	protected RGBIndicator rgbIndicator;

	// Button state tracking to prevent continuous input
	protected boolean leftTriggerPressed = false;
	protected boolean rightTriggerPressed = false;
	protected boolean xButtonPressed = false;
	protected boolean aButtonPressed = false;
	protected boolean bButtonPressed = false;
	protected boolean spindexerUpCrossed = false;
	protected boolean spindexerMidCrossed = false;
	protected boolean spindexerDownCrossed = false;
	protected boolean dpadUpPressed = false;
	protected boolean dpadDownPressed = false;
	protected boolean transferAboveRPM = false;
	protected int lastSpindexerTarget = 0;

	// Performance monitoring
	private long lastLoopTime = 0;
	private long maxLoopTime = 0;
	private long loopCount = 0;
	private static final long TARGET_LOOP_TIME_NS = 5_000_000; // 5ms target

	@Override
	public void init() {
		// Initialize hardware
		HardwareInitializer.initialize(hardwareMap);
		drive = new MecanumDrive(hardwareMap, getStartingPose());
		scheduler = ActionScheduler.getInstance();
		shooter = Shooter.getInstance();
		intake = Intake.getInstance();
		transfer = Transfer.getInstance();
		spindexer = Spindexer.getInstance();
		spindexer.resetCalibrationAverage();
		rgbIndicator = RGBIndicator.getInstance();
		limelight = new Limelight(hardwareMap);

		telemetry.addData("Status", "Initialized - Waiting for START");
		telemetry.update();
	}

	@Override
	public void init_loop() {
		spindexer.updateCalibrationAverage();
	}

	@Override
	public void start() {
		// Called when START is pressed
		spindexer.finalizeTeleOpCalibration();
		scheduler.schedule(transfer.intakeDoorForward());
		scheduler.schedule(transfer.transferBackward());
		scheduler.update();
		limelight.start();

		scheduler.schedule(spindexer.setTarget(0));
	}

	@Override
	public void loop() {
		long startTime = System.nanoTime();

		// CRITICAL - Must complete quickly for responsive driving
		drive.updatePoseEstimate();
		limelight.update();
		handleDriveInput();

		// Phase 1: Core subsystem updates
        SubsystemUpdater.update();
        handleOperatorInput();
        spindexer.update();
		scheduler.update();

		// Phase 2: Non-critical updates (budget: 4.5ms)
		if ((System.nanoTime() - startTime) < 30_000_000) {
			updateRGBIndicator();
			displayTelemetry();
			telemetry.update();
		}

		// Performance monitoring
		long loopTime = System.nanoTime() - startTime;
		if (loopTime > maxLoopTime) {
			maxLoopTime = loopTime;
		}
		loopCount++;

		// Log performance stats periodically
		if (loopCount % 100 == 0) {
			telemetry.addData("Performance", "Max loop time: %.2fms", maxLoopTime / 1_000_000.0);
			telemetry.addData("Performance", "Avg loop time: %.2fms", (System.nanoTime() - lastLoopTime) / 1_000_000.0 / 100.0);
			lastLoopTime = System.nanoTime();
		}
	}

	@Override
	public void stop() {
		// Called when OpMode is stopped
		HardwareShutdown.shutdown();
	}

	/**
	 * Override this method in subclasses to set the starting pose
	 */
	protected Pose2d getStartingPose() {
		return new Pose2d(0, 0, 0);
	}

	protected Team getTeam() {
		return Team.UNKNOWN;
	}

	/**
	 * Update RGB indicator color based on shooter RPM using discrete color ranges.
	 * Predefined colors: Off, Red, Orange, Yellow, Sage, Green, Azure, Blue, Indigo, Violet, White
	 */
	private void updateRGBIndicator() {
		double rpm = shooter.averageRPM;
		double color = 0;

		if (rpm <= 1000) {
			color = 0.277;
		} else if (rpm <= 1500) {
			color = 0.333;
		} else if (rpm <= 2000) {
			color = 0.388;
		} else if (rpm <= 2400) {
			color = 0.444;
		} else if (rpm <= 2600) {
			color = 0.500;
		} else if (rpm <= 2800) {
			color = 0.666;
		} else if (rpm > 2800) {
			color = 0.722;
		}

		rgbIndicator.setDirectPosition(color);
	}

	/**
	 * Handle driving input from gamepad1
	 */
	private void handleDriveInput() {
		if (gamepad1.a && !aButtonPressed) {
			if (getTeam() == Team.RED) {
				scheduler.schedule(
						drive.actionBuilder(new Pose2d(limelight.botPose.getPosition().x, limelight.botPose.getPosition().y, limelight.botPose.getOrientation().getYaw()))
								.strafeToLinearHeading(new Vector2d(56.75, 10.25),Math.toRadians(-23))
								.build()
				);
			} else if (getTeam() == Team.BLUE) {
				scheduler.schedule(
						drive.actionBuilder(new Pose2d(limelight.botPose.getPosition().x, limelight.botPose.getPosition().y, limelight.botPose.getOrientation().getYaw()))
								.strafeToLinearHeading(new Vector2d(56.75, -10.25),Math.toRadians(23))
								.build()
				);
			}
			aButtonPressed = true;
		} else if (!gamepad1.a && aButtonPressed) {
			aButtonPressed = false;
		}

		if (!gamepad1.a) {
			double forwardPower;
			double turnPower;
			double strafePower;

			if (gamepad1.right_bumper){
				forwardPower = -gamepad1.left_stick_y / 2; // Left stick Y (inverted)
				turnPower = -gamepad1.right_stick_x / 2; // Right stick X
				strafePower = -gamepad1.left_stick_x / 2; // Left stick X
			}else{
				 forwardPower = -gamepad1.left_stick_y; // Left stick Y (inverted)
				 turnPower = -gamepad1.right_stick_x; // Right stick X
				 strafePower = -gamepad1.left_stick_x; // Left stick X
			}

			// Apply deadzone
			forwardPower = Math.abs(forwardPower) > 0.05 ? forwardPower : 0;
			strafePower = Math.abs(strafePower) > 0.05 ? strafePower : 0;
			turnPower = Math.abs(turnPower) > 0.05 ? turnPower : 0;

			// Create velocity command
			PoseVelocity2d velocity = new PoseVelocity2d(
					new Vector2d(forwardPower, strafePower),
					turnPower
			);
			drive.setDrivePowers(velocity);
		}

//		// Manual offset trim
//		if (gamepad1.left_bumper && !leftBumperPressed) {
//			scheduler.schedule(spindexer.applyManualOffsetTrim(-1.0));
//			leftBumperPressed = true;
//		} else if (!gamepad1.left_bumper && leftBumperPressed) {
//			leftBumperPressed = false;
//		}
//
//		if (gamepad1.right_bumper && !rightBumperPressed) {
//			scheduler.schedule(spindexer.applyManualOffsetTrim(1.0));
//			rightBumperPressed = true;
//		} else if (!gamepad1.right_bumper && rightBumperPressed) {
//			rightBumperPressed = false;
//		}
	}

	/**
	 * Handle operator controls from gamepad2
	 */
	protected void handleOperatorInput() {
		// Left Trigger: run intake
		if (gamepad2.left_trigger > 0.5 && !leftTriggerPressed) {
			scheduler.schedule(intake.in());
			leftTriggerPressed = true;
		} else if (gamepad2.left_trigger <= 0.5 && leftTriggerPressed) {
			scheduler.schedule(intake.stop());
			leftTriggerPressed = false;
		}

		// Right Trigger: schedule Shooter.run() repeatedly while pressed, Shooter.stop() once when released
		if (gamepad2.right_trigger > 0.5) {
			scheduler.schedule(shooter.run(Shooter.AUDIENCE_RPM));
			rightTriggerPressed = true;
		} else if (gamepad2.right_trigger <= 0.5 && rightTriggerPressed) {
			scheduler.schedule(shooter.stop());
			rightTriggerPressed = false;
		}

		// X Button: Override transfer forward - manual control
		if (gamepad2.x && !xButtonPressed) {
			scheduler.schedule(transfer.transferForward());
			xButtonPressed = true;
		} else if (!gamepad2.x && xButtonPressed) {
			scheduler.schedule(transfer.transferBackward());
			xButtonPressed = false;
		}

		// Automatic transfer based on readiness (only if neither X nor Y button held)
		if (!gamepad2.x && !gamepad2.y) {
			boolean isReady = TransferUtility.isTransferReady(spindexer, shooter, Shooter.AUDIENCE_RPM);
			if (isReady && !transferAboveRPM) {
				scheduler.schedule(transfer.transferForward());
			} else if (!isReady && transferAboveRPM) {
				scheduler.schedule(transfer.transferBackward());
			}
			transferAboveRPM = isReady;
		}

		// B Button: Intake door backward and intake out when pressed, forward and intake stop when released
		if (gamepad2.b && !bButtonPressed) {
			scheduler.schedule(transfer.intakeDoorBackward());
			scheduler.schedule(intake.out());
			bButtonPressed = true;
		} else if (!gamepad2.b && bButtonPressed) {
			scheduler.schedule(transfer.intakeDoorForward());
			scheduler.schedule(intake.stop());
			bButtonPressed = false;
		}

		// Left joystick: Spindexer control with threshold crossing (inverted Y axis)
		double leftJoystickY = -gamepad2.left_stick_y;

		// Dead zone: stop spindexer
		if (leftJoystickY > -0.2 && leftJoystickY < 0.2) {
			if (!spindexerMidCrossed) {
				scheduler.schedule(spindexer.setDirectPower(0));
				spindexerMidCrossed = true;
				spindexerUpCrossed = false;
				spindexerDownCrossed = false;
			}
		}
		// Crosses 0.2 threshold going up (from lower to 0.2+)
		else if (leftJoystickY >= 0.2 && !spindexerUpCrossed) {
			scheduler.schedule(spindexer.setDirectPower(0.25));
			spindexerUpCrossed = true;
			spindexerMidCrossed = false;
			spindexerDownCrossed = false;
		}
		// Crosses -0.2 threshold going down (to -0.2 or below)
		else if (leftJoystickY <= -0.2 && !spindexerDownCrossed) {
			scheduler.schedule(spindexer.setDirectPower(-0.25));
			spindexerDownCrossed = true;
			spindexerMidCrossed = false;
			spindexerUpCrossed = false;
		}

		if (gamepad2.dpad_down && !dpadDownPressed) {
			double nextIntakePosition = SpindexerPositionUtility.getNextIntakePosition(lastSpindexerTarget);
			lastSpindexerTarget = (int) nextIntakePosition;
			scheduler.schedule(spindexer.setTarget(nextIntakePosition));
			dpadDownPressed = true;
		} else if (!gamepad2.dpad_down && dpadDownPressed) {
			dpadDownPressed = false;
		}

		if (gamepad2.dpad_up && !dpadUpPressed) {
			double nextShootPosition = SpindexerPositionUtility.getNextShootPosition(lastSpindexerTarget);
			lastSpindexerTarget = (int) nextShootPosition;
			scheduler.schedule(spindexer.setTarget(nextShootPosition));
			dpadUpPressed = true;
		} else if (!gamepad2.dpad_up && dpadUpPressed) {
			dpadUpPressed = false;
		}
	}

	/**
	 * Display telemetry information
	 */
	protected void displayTelemetry() {
		telemetry.addLine("=== MAIN TELEOP ===");
		telemetry.addData("Drive Mode", "Mecanum");

		telemetry.addLine("=== GAMEPAD 1 (Driver) ===");
		telemetry.addData("Forward", String.format("%.2f", -gamepad1.left_stick_y));
		telemetry.addData("Strafe", String.format("%.2f", gamepad1.left_stick_x));
		telemetry.addData("Turn", String.format("%.2f", gamepad1.right_stick_x));

		telemetry.addData("Location", drive.localizer.getPose().toString());

		telemetry.addLine("=== GAMEPAD 2 (Operator) ===");
		telemetry.addData("Left Trigger", "Intake");
		telemetry.addData("Right Trigger", "Shooter");
		telemetry.addData("Left Joystick Y (Spindexer)", String.format("%.2f", -gamepad2.left_stick_y));
		telemetry.addData("Spindexer Position", String.format("%.2f", spindexer.getCalibratedPosition() / 360.0));

		telemetry.addLine("=== SHOOTER ===");
		telemetry.addData("Upper RPM", String.format("%.2f", shooter.upperRPM));
		telemetry.addData("Lower RPM", String.format("%.2f", shooter.lowerRPM));
		telemetry.addData("Average RPM", String.format("%.2f", shooter.averageRPM));

		telemetry.addLine("=== Spindexer ===");
		telemetry.addData("Current Location", spindexer.getCalibratedPosition());
		telemetry.addData("Target", spindexer.targetPosition);
		telemetry.addData("Next intake", SpindexerPositionUtility.getNextIntakePosition((int) spindexer.getCalibratedPosition()));
		telemetry.addData("Next shoot", SpindexerPositionUtility.getNextShootPosition((int) spindexer.getCalibratedPosition()));

		telemetry.addLine("=== Transfer ===");
		telemetry.addData("Spindexer at Shooting Pos", TransferUtility.isSpindexerAtShootingPosition(spindexer));
		telemetry.addData("Shooter at Target RPM", TransferUtility.isShooterAtTargetRPM(shooter, Shooter.AUDIENCE_RPM));
		telemetry.addData("Transfer Ready", TransferUtility.isTransferReady(spindexer, shooter, Shooter.AUDIENCE_RPM));
	}
}
