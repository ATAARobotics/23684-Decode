package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import static org.firstinspires.ftc.teamcode.Utils.GamepadUtils.applyDeadzone;
import static org.firstinspires.ftc.teamcode.Utils.GamepadUtils.inDeadzone;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Limelight;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;
import org.firstinspires.ftc.teamcode.Subsystem.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystem.Transfer;
import org.firstinspires.ftc.teamcode.Utils.ShootAngle;
import org.firstinspires.ftc.teamcode.Utils.Team;

@Config
@Configurable
@TeleOp
public class MainTeleOp extends OpMode {
	protected Follower follower;
	protected CommandScheduler scheduler;
	protected Shooter shooter;
	protected Intake intake;
	protected Transfer transfer;
	protected Spindexer spindexer;
	protected Limelight limelight;
	protected TelemetryManager.TelemetryWrapper panelsTelemetry;

	// Button state tracking to prevent continuous input
	protected boolean leftTriggerPressed = false;
	protected boolean rightTriggerPressed = false;
	protected boolean xButtonPressed = false;
	protected boolean aButtonPressed = false;
	protected boolean bButtonPressed = false;
	protected boolean dpadUpPressed = false;
	protected boolean dpadDownPressed = false;
	protected boolean spindexerUpCrossed = false;
	protected boolean spindexerMidCrossed = false;
	protected boolean spindexerDownCrossed = false;
	DcMotorEx frontRight;
	DcMotorEx rearRight;
	DcMotorEx frontLeft;
	DcMotorEx rearLeft;
	ElapsedTime timer = new ElapsedTime();
	private Servo rgbServo;
	public static double spindexerPower = 0.5;

	// Performance monitoring
	private long maxLoopTime = 0;

	double indicatorValue() {
		// TODO: Export to a util class and beautify
		double x = timer.seconds();
		double hz = 1;
		if (hardwareMap.voltageSensor.iterator().next().getVoltage() >= 13.5) {
			hz = 2.5;
		} else if (hardwareMap.voltageSensor.iterator().next().getVoltage() <= 13.5 && hardwareMap.voltageSensor.iterator().next().getVoltage() >= 12) {
			hz = 1;
		} else if (hardwareMap.voltageSensor.iterator().next().getVoltage() <= 11) {
			hz = 0.5;
		} else {
			hz = 0.5;
		}
		int state = Math.floorMod((int) Math.floor(x * hz), 2);
		return 0.23 * state + 0.388;
	}

	@Override
	public void init() {
		// Initialize hardware
		follower = Constants.createFollower(hardwareMap);
		follower.setStartingPose(getStartingPose());
		scheduler = CommandScheduler.getInstance();
		scheduler.reset();
		scheduler.setBulkReading(hardwareMap, LynxModule.BulkCachingMode.AUTO);
		shooter = new Shooter(hardwareMap);
		intake = new Intake(hardwareMap);
		transfer = new Transfer(hardwareMap);
		spindexer = new Spindexer(hardwareMap);
		// Set shooter dependency for conditional transfer
		transfer.setShooter(shooter);
		transfer.setSpindexer(spindexer);
		// TODO: Make subsystem
		rgbServo = hardwareMap.get(Servo.class, "rgbIndicator");
		limelight = new Limelight(hardwareMap);

		panelsTelemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();

		// TODO: Make subsystem
		frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
		rearRight = hardwareMap.get(DcMotorEx.class, "rearRight");
		frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
		rearLeft = hardwareMap.get(DcMotorEx.class, "rearLeft");

		frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		rearRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		rearLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

		telemetry.addData("Status", "Initialized - Waiting for START");
		telemetry.update();
	}

	@Override
	public void start() {
		// Called when START is pressed
//		limelight.start();
		timer.startTime();
		scheduler.run();
	}

	@Override
	public void init_loop() {
		scheduler.run();
	}

	@Override
	public void loop() {
		long startTime = System.nanoTime();

		// CRITICAL - Must complete quickly for responsive driving
		follower.update();
//		scheduler.schedule(limelight.update());
		handleDriveInput();
		handleOperatorInput();
		updateRGBIndicator();
		scheduler.run();
		shooter.periodic();

		displayTelemetry();

		// Performance monitoring
		long loopTime = System.nanoTime() - startTime;

		if (loopTime > maxLoopTime) {
			maxLoopTime = loopTime;
		}

		// ALWAYS log performance stats to driver station (critical for monitoring during competition)
		// This is separate from dashboard telemetry and always visible to the driver
		telemetry.addData("Loop Time", "Current: %.2fms", loopTime / 1_000_000.0);
		telemetry.addData("Loop Time", "Max: %.2fms", maxLoopTime / 1_000_000.0);

		telemetry.update();
	}

	@Override
	public void stop() {
		// Called when OpMode is stopped
		// TODO: Stop all motors and disable the follower and scheduler
	}

	/**
	 * Override this method in subclasses to set the starting pose
	 */
	protected Pose getStartingPose() {
		return new Pose(63.000, 9, Math.toRadians(270));
	}

	protected Team getTeam() {
		return Team.BLUE;
	}

	/**
	 * Update RGB indicator color
	 */
	private void updateRGBIndicator() {
		rgbServo.setPosition(indicatorValue());
//        scheduler.schedule(rgbIndicator.setColorAction(Transfer.isShooterAtTargetRPM(shooter, Shooter.AUDIENCE_RPM) ? "#00AB66" : "#FF2C2C"));
	}

	/**
	 * Handle driving input from gamepad1
	 */
	private void handleDriveInput() {
		if (gamepad1.a && !aButtonPressed) {
			if (getTeam() == Team.RED) {
				follower.turnTo(ShootAngle.calculateShotAngle(follower.getPose().getX(), follower.getPose().getY(), 144, 144));
			} else if (getTeam() == Team.BLUE) {
				follower.turnTo(ShootAngle.calculateShotAngle(follower.getPose().getX(), follower.getPose().getY(), 0, 144));
			}
			aButtonPressed = true;
		} else if (!gamepad1.a && aButtonPressed) {
			aButtonPressed = false;
		}

		if (!gamepad1.a) {
			if (!follower.isTeleopDrive()){
				follower.startTeleOpDrive(true);
			}

			follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
		}
	}

	/**
	 * Handle operator controls from gamepad2
	 */
	protected void handleOperatorInput() {
		// Left Trigger: run intake
		if (gamepad2.left_trigger > 0.5 && !leftTriggerPressed) {
			scheduler.schedule(intake.In());
			scheduler.schedule(transfer.IntakeDoorOut());
			leftTriggerPressed = true;
		} else if (gamepad2.left_trigger <= 0.5 && leftTriggerPressed) {
			scheduler.schedule(intake.Stop());
			scheduler.schedule(transfer.IntakeDoorStop());
			leftTriggerPressed = false;
		}

		// Right Trigger: Shooter with conditional transfer (only when trigger held)
		if (gamepad2.right_trigger > 0.5 && !rightTriggerPressed) {
			shooter.setTarget(Shooter.AUDIENCE_RPM, Shooter.AUDIENCE_RPM);
			rightTriggerPressed = true;
		} else if (gamepad2.right_trigger <= 0.5 && rightTriggerPressed) {
			shooter.setTarget(0, 0);
			scheduler.schedule(transfer.TransferStop());
			rightTriggerPressed = false;
		}

		// X Button: Override transfer forward - manual control
		if (gamepad2.x && !xButtonPressed) {
			scheduler.schedule(transfer.TransferOut());
			xButtonPressed = true;
		} else if (!gamepad2.x && xButtonPressed) {
			scheduler.schedule(transfer.TransferStop());
			xButtonPressed = false;
		}

		if (!gamepad2.x) {
			transfer.updateAutomaticTransfer(!rightTriggerPressed);
		}

		// B Button: Intake door backward and intake out when pressed, forward and intake stop when released
		if (gamepad2.b && !bButtonPressed) {
			scheduler.schedule(transfer.IntakeDoorIn());
			scheduler.schedule(intake.Out());
			bButtonPressed = true;
		} else if (!gamepad2.b && bButtonPressed) {
			scheduler.schedule(transfer.IntakeDoorStop());
			scheduler.schedule(intake.Stop());
			bButtonPressed = false;
		}

		if (gamepad2.dpad_up && !dpadUpPressed) {
			scheduler.schedule(spindexer.NextTarget());
			dpadUpPressed = true;
		} else if (!gamepad2.dpad_up && dpadUpPressed) {
			dpadUpPressed = false;
		}

		// Left joystick: Spindexer control with threshold crossing (inverted Y axis)
		double leftJoystickY = -gamepad2.left_stick_y;

		// Dead zone: stop spindexer
		if (leftJoystickY > -0.2 && leftJoystickY < 0.2) {
			if (!spindexerMidCrossed) {
				scheduler.schedule(spindexer.DirectPower(0));
				scheduler.schedule(transfer.IntakeDoorStop());
				if (!gamepad2.x) {
					scheduler.schedule(transfer.TransferStop());
				}
				spindexerMidCrossed = true;
				spindexerUpCrossed = false;
				spindexerDownCrossed = false;
			}
		}

		// Crosses 0.2 threshold going up (from lower to 0.2+)
		if (leftJoystickY >= 0.2 && !spindexerUpCrossed) {
			scheduler.schedule(spindexer.DirectPower(spindexerPower));
			scheduler.schedule(transfer.IntakeDoorOut());
			if (!gamepad2.x) {
				scheduler.schedule(transfer.TransferIn());
			}
			spindexerUpCrossed = true;
			spindexerMidCrossed = false;
			spindexerDownCrossed = false;
		}

		// Crosses -0.2 threshold going down (to -0.2 or below)
		else if (leftJoystickY <= -0.2 && !spindexerDownCrossed) {
			scheduler.schedule(spindexer.DirectPower(-spindexerPower));
			scheduler.schedule(transfer.IntakeDoorIn());
			if (!gamepad2.x) {
				scheduler.schedule(transfer.TransferIn());
			}
			spindexerDownCrossed = true;
			spindexerMidCrossed = false;
			spindexerUpCrossed = false;
		}
	}

	/**
	 * Display telemetry information
	 */
	protected void displayTelemetry() {
		panelsTelemetry.addLine("=== MAIN TELEOP ===");
		panelsTelemetry.addData("Drive Mode", "Mecanum");
		panelsTelemetry.addData("Location", follower.getPose().toString());

		panelsTelemetry.addLine("=== SHOOTER ===");
		panelsTelemetry.addData("Upper RPM", shooter.upperRPM);
		panelsTelemetry.addData("Lower RPM", shooter.lowerRPM);
		panelsTelemetry.addData("Average RPM", shooter.averageRPM);

		panelsTelemetry.addLine("=== TRANSFER ===");
		panelsTelemetry.addData("Shooter Lower At Target (This may be inactive, you may need to refer to \"At Target\")", transfer.reachedLowerTarget);
		panelsTelemetry.addData("Shooter Upper At Target (This may be inactive, you may need to refer to \"At Target\")", transfer.reachedUpperTarget);
		panelsTelemetry.addData("Shooter At Target (This may be inactive, you may need to refer to \"Lower At Target\" and \"Upper At Target\")", transfer.reachedAverageTarget);
		panelsTelemetry.addData("Spindexer At Target", transfer.spindexerAtTarget);

		panelsTelemetry.update();
	}
}