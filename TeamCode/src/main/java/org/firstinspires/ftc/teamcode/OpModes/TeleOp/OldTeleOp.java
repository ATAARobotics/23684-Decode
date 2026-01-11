package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import static org.firstinspires.ftc.teamcode.Utils.GamepadUtils.applyDeadzone;
import static org.firstinspires.ftc.teamcode.Utils.GamepadUtils.inDeadzone;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
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

import java.util.List;

@Config
@Configurable
@TeleOp
public class OldTeleOp extends OpMode {
	private static final long TELEMETRY_UPDATE_INTERVAL = 100_000_000; // 100ms (roughly 100 loop ticks at 10ms/loop)
	private static final long MAX_LOOP_TIME_FOR_TELEMETRY = 25_000_000; // 25ms
	// Telemetry throttling
	public static boolean ENABLE_TELEMETRY = false;
	protected Follower follower;
	protected CommandScheduler scheduler;
	protected Shooter shooter;
	protected Intake intake;
	protected Transfer transfer;
	protected Spindexer spindexer;
	protected Limelight limelight;
	protected List<LynxModule> allHubs;
	// Button state tracking to prevent continuous input
	protected boolean leftTriggerPressed = false;
	protected boolean rightTriggerPressed = false;
	protected boolean xButtonPressed = false;
	protected boolean aButtonPressed = false;
	protected boolean bButtonPressed = false;
	protected boolean spindexerUpCrossed = false;
	protected boolean spindexerMidCrossed = false;
	protected boolean spindexerDownCrossed = false;
	DcMotorEx frontRight;
	DcMotorEx rearRight;
	DcMotorEx frontLeft;
	DcMotorEx rearLeft;
	ElapsedTime timer = new ElapsedTime();
	private Servo rgbServo;
	// Performance monitoring
	private long maxLoopTime = 0;
	private final long lastTelemetryTime = 0;
	private long previousLoopTime = 0;

	double indicatorValue() {
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
		shooter = new Shooter(hardwareMap);
		intake = new Intake(hardwareMap);
		transfer = new Transfer(hardwareMap);
		spindexer = new Spindexer(hardwareMap);
		// TODO: Make subsystem
		rgbServo = hardwareMap.get(Servo.class, "rgbIndicator");
		limelight = new Limelight(hardwareMap);

		// TODO: Make subsystem
		frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
		rearRight = hardwareMap.get(DcMotorEx.class, "rearRight");
		frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
		rearLeft = hardwareMap.get(DcMotorEx.class, "rearLeft");

		frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		rearRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		rearLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

		// Enable Lynx bulk caching to reduce USB latency
		allHubs = hardwareMap.getAll(LynxModule.class);
		for (LynxModule hub : allHubs) {
			hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
		}

		telemetry.addData("Status", "Initialized - Waiting for START");
		telemetry.addData("Bulk Caching", "MANUAL mode enabled on " + allHubs.size() + " hub(s)");
		telemetry.update();
	}

	@Override
	public void start() {
		// Called when START is pressed
		scheduler.run();
		limelight.start();
		timer.startTime();
	}

	@Override
	public void init_loop() {
		for (LynxModule hub : allHubs) {
			hub.clearBulkCache();
		}
	}

	@Override
	public void loop() {
		long startTime = System.nanoTime();

		for (LynxModule hub : allHubs) {
			hub.clearBulkCache();
		}

		// CRITICAL - Must complete quickly for responsive driving
		follower.update();
		limelight.update();
		handleDriveInput();
		handleOperatorInput();
		shooter.periodic();
		updateRGBIndicator();

		// Non-critical updates (budget: 25ms)
		if (ENABLE_TELEMETRY && (System.nanoTime() - startTime) < 25_000_000) {
			displayTelemetry();
		}

		// Performance monitoring
		long loopTime = System.nanoTime() - startTime;
		previousLoopTime = loopTime;

		if (loopTime > maxLoopTime) {
			maxLoopTime = loopTime;
		}

		// ALWAYS log performance stats to driver station (critical for monitoring during competition)
		// This is separate from dashboard telemetry and always visible to the driver
		telemetry.addData("Loop Time", "Current: %.2fms", loopTime / 1_000_000.0);
		telemetry.addData("Loop Time", "Max: %.2fms", maxLoopTime / 1_000_000.0);

		if (ENABLE_TELEMETRY) {
			telemetry.addData("Telemetry", "Dashboard enabled");
		} else {
			telemetry.addData("Telemetry", "Dashboard disabled (low latency)");
		}

		telemetry.update();
	}

	@Override
	public void stop() {
		// Called when OpMode is stopped
	}

	/**
	 * Override this method in subclasses to set the starting pose
	 */
	protected Pose getStartingPose() {
		return new Pose(0, 0, 0);
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
			if (inDeadzone(gamepad1.left_stick_y) && inDeadzone(gamepad1.left_stick_x) && inDeadzone(gamepad1.right_stick_x)) {
				// No driver movement
				follower.holdPoint(follower.getPose());
				scheduler.schedule(transfer.IntakeDoorOut());
			} else {
				if (follower.isBusy()) {
					follower.breakFollowing();
				}

				double y = applyDeadzone(-gamepad1.left_stick_y); // Y stick value is reversed
				double x = applyDeadzone(gamepad1.left_stick_x * 1.1); // Counteract imperfect strafing
				double rx = applyDeadzone(gamepad1.right_stick_x);

				// Denominator is the largest motor power (absolute value) or 1
				// This ensures all the powers maintain the same ratio,
				// but only if at least one is out of the range [-1, 1]
				double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
				double frontLeftPower = (y + x + rx) / denominator;
				double backLeftPower = (y - x + rx) / denominator;
				double frontRightPower = (y - x - rx) / denominator;
				double backRightPower = (y + x - rx) / denominator;

				frontLeft.setPower(frontLeftPower);
				rearLeft.setPower(backLeftPower);
				frontRight.setPower(frontRightPower);
				rearRight.setPower(backRightPower);
			}
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

		// Right Trigger: schedule Shooter.run() repeatedly while pressed, Shooter.stop() once when released
		if (gamepad2.right_trigger > 0.5) {
			shooter.setTarget(Shooter.AUDIENCE_RPM, Shooter.AUDIENCE_RPM);
			rightTriggerPressed = true;
		} else if (gamepad2.right_trigger <= 0.5 && rightTriggerPressed) {
			shooter.setTarget(0, 0);
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

		// Automatic transfer based on readiness (only if X button isn't held)
//        if (!gamepad2.x) {
//            boolean isReady = Transfer.isTransferReady(spindexer, shooter, Shooter.AUDIENCE_RPM);
//            if (isReady && !transferAboveRPM) {
//                scheduler.schedule(transfer.transferIn());
//            } else if (!isReady && transferAboveRPM) {
//                scheduler.schedule(transfer.transferOut());
//            }
//            transferAboveRPM = isReady;
//        }

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

		double spindexerPower;

		if (gamepad2.right_bumper) {
			spindexerPower = 0.31;
		} else {
			spindexerPower = 0.37;
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
		telemetry.addLine("=== MAIN TELEOP ===");
		telemetry.addData("Drive Mode", "Mecanum");

		telemetry.addLine("=== GAMEPAD 1 (Driver) ===");
		telemetry.addData("Forward", String.format("%.2f", -gamepad1.left_stick_y));
		telemetry.addData("Strafe", String.format("%.2f", gamepad1.left_stick_x));
		telemetry.addData("Turn", String.format("%.2f", gamepad1.right_stick_x));

		telemetry.addData("Location", follower.getPose().toString());

		telemetry.addLine("=== GAMEPAD 2 (Operator) ===");
		telemetry.addData("Left Trigger", "Intake");
		telemetry.addData("Right Trigger", "Shooter");
		telemetry.addData("Left Joystick Y (Spindexer)", String.format("%.2f", -gamepad2.left_stick_y));

		telemetry.addLine("=== SHOOTER ===");
		telemetry.addData("Upper RPM", String.format("%.2f", shooter.upperRPM));
		telemetry.addData("Lower RPM", String.format("%.2f", shooter.lowerRPM));
		telemetry.addData("Average RPM", String.format("%.2f", shooter.averageRPM));

		telemetry.addLine("=== Transfer ===");
//        telemetry.addData("Spindexer at Shooting Pos", Transfer.isSpindexerAtShootingPosition(spindexer));
//        telemetry.addData("Shooter at Target RPM", Transfer.isShooterAtTargetRPM(shooter, Shooter.AUDIENCE_RPM));
//        telemetry.addData("Transfer Ready", Transfer.isTransferReady(spindexer, shooter, Shooter.AUDIENCE_RPM));
	}
}