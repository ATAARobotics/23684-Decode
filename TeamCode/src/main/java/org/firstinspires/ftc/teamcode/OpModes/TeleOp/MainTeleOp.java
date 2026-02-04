package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Limelight;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;
import org.firstinspires.ftc.teamcode.Subsystem.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystem.Transfer;
import org.firstinspires.ftc.teamcode.Utils.RobotPosition;
import org.firstinspires.ftc.teamcode.Utils.ShootAngle;
import org.firstinspires.ftc.teamcode.Utils.Team;

import java.util.function.Supplier;

@Config
@Configurable
public abstract class MainTeleOp extends OpMode {
	public static double spindexerPower = 0.5;
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
	protected boolean b1ButtonPressed = false;
	protected boolean b2ButtonPressed = false;
	protected boolean dpadUpPressed = false;
	protected boolean dpadDownPressed = false;
	protected boolean yButtonPressed = false;
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

	private Supplier<PathChain> pathBackBlue;

	private Supplier<PathChain> pathFrontBlue;

	private Supplier<PathChain> redAudienceShootingPath;

	double upperShooterSpeed = 0;
	double lowershooterSpeed = 0;



//	double indicatorValue() {
//		// TODO: Export to a util class and beautify
//		double x = timer.seconds();
//		double hz = 1;
//		if (hardwareMap.voltageSensor.iterator().next().getVoltage() >= 13.5) {
//			hz = 2.5;
//		} else if (hardwareMap.voltageSensor.iterator().next().getVoltage() <= 13.5 && hardwareMap.voltageSensor.iterator().next().getVoltage() >= 12) {
//			hz = 1;
//		} else if (hardwareMap.voltageSensor.iterator().next().getVoltage() <= 11) {
//			hz = 0.5;
//		} else {
//			hz = 0.5;
//		}
//		int state = Math.floorMod((int) Math.floor(x * hz), 2);
//		return 0.23 * state + 0.388;
//	}

	@Override
	public void init() {
		// Initialize hardware
		follower = Constants.createFollower(hardwareMap);
		if (RobotPosition.isPoseSet) {
			follower.setStartingPose(RobotPosition.robotPose);
			RobotPosition.isPoseSet = false;
		} else {
			follower.setStartingPose(getStartingPose());
		}
		scheduler = CommandScheduler.getInstance();
		scheduler.reset();
		limelight = new Limelight(hardwareMap);
		scheduler.setBulkReading(hardwareMap, LynxModule.BulkCachingMode.AUTO);
		shooter = new Shooter(hardwareMap);
		scheduler.schedule(shooter.SetTarget(0, 0));
		intake = new Intake(hardwareMap);
		transfer = new Transfer(hardwareMap);
		spindexer = new Spindexer(hardwareMap);
		// Set shooter dependency for conditional transfer
		transfer.setShooter(shooter);
		transfer.setSpindexer(spindexer);
		// TODO: Make subsystem
		rgbServo = hardwareMap.get(Servo.class, "rgbIndicator");
		// limelight = new Limelight(hardwareMap);

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

		pathBackBlue = () -> follower.pathBuilder()
				.addPath(new Path(new BezierLine(follower::getPose, new Pose(65, 11))))
				.setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(295), 0.8))
				.build();

		pathFrontBlue = () -> follower.pathBuilder()
				.addPath(new Path(new BezierLine(follower::getPose, new Pose(70, 103))))
				.setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading,Math.toRadians(334), 0.8))
				.build();

		redAudienceShootingPath = () -> follower.pathBuilder()
				.addPath(new Path(new BezierLine(follower::getPose, new Pose(79, 11))))
				.setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(245), 0.8))
				.build();

		scheduler.run();
	}

	@Override
	public void init_loop() {
		scheduler.run();
	}

	@Override
	public void start() {
		// Called when START is pressed
//		limelight.start();
		spindexer.zeroSpindexer();
		timer.startTime();
		scheduler.run();
	}

	@Override
	public void loop() {
		long startTime = System.nanoTime();

		// CRITICAL - Must complete quickly for responsive driving
		follower.update();
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

		limelight.updateLowPass(follower.getHeading());
//		if (limelight.goalsFound() && (follower.isTeleopDrive() || !follower.isBusy())) {
//			Pose botPose = limelight.PPVisionPoseRaw();
//
//			follower.setPose(new Pose(
//					botPose.getX(),
//					botPose.getY(),
//					follower.getHeading()
//			));
//		}


		if (gamepad1.xWasPressed()){
			gamepad1.rumble(300);
			follower.setPose(new Pose(142.7202744371309, 7.36770930252676, 0));
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
	protected abstract Pose getStartingPose();

	protected abstract Team getTeam();

	/**
	 * Update RGB indicator color
	 */
	private void updateRGBIndicator() {
		if (rightTriggerPressed) {
			rgbServo.setPosition(transfer.spindexerAtTarget && transfer.reachedAverageTarget ? 0.50 : 0.28);
		} else {
			rgbServo.setPosition(0.65);
		}
	}

	/**
	 * Handle driving input from gamepad1
	 */
	private void handleDriveInput() {
		if (gamepad1.cross && !aButtonPressed) {
			upperShooterSpeed = Shooter.AUDIENCE_RPM;
			lowershooterSpeed = Shooter.AUDIENCE_RPM;
			if (getTeam() == Team.RED) {
				follower.followPath(redAudienceShootingPath.get(), true);
			} else if (getTeam() == Team.BLUE) {
				follower.followPath(pathBackBlue.get(), true);
			}
			aButtonPressed = true;
		} else if (!gamepad1.cross && aButtonPressed) {
			aButtonPressed = false;
		}
		else if (gamepad1.circle && !b1ButtonPressed) {
			upperShooterSpeed = Shooter.GOAL_RPM_UPPER;
			lowershooterSpeed = Shooter.GOAL_RPM_UPPER;

			if (getTeam() == Team.RED) {
				follower.turnTo(ShootAngle.calculateShotAngle(follower.getPose().getX(), follower.getPose().getY(), 144, 144));
			} else if (getTeam() == Team.BLUE) {
				follower.followPath(pathFrontBlue.get(), true);
			}
			b1ButtonPressed = true;
		} else if (!gamepad1.circle && b1ButtonPressed) {
			b1ButtonPressed = false;
		}

		if (!gamepad1.circle && !gamepad1.cross) {
			if (!follower.isTeleopDrive()) {
				follower.startTeleOpDrive(true);
			}

			follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
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
			shooter.setTarget(upperShooterSpeed,lowershooterSpeed);
			rightTriggerPressed = true;
		} else if (gamepad2.right_trigger <= 0.5 && rightTriggerPressed) {
			shooter.setTarget(0, 0);
			scheduler.schedule(transfer.TransferStop());
			rightTriggerPressed = false;
		}

		// X Button: Override transfer forward - manual control
		if (gamepad2.y && !xButtonPressed) {
			scheduler.schedule(transfer.TransferOut());
			xButtonPressed = true;
		} else if (!gamepad2.y && xButtonPressed) {
			scheduler.schedule(transfer.TransferStop());
			xButtonPressed = false;
		}

		if (!gamepad2.y) {
			transfer.updateAutomaticTransfer(!rightTriggerPressed);
		}

		// B Button: Intake door backward and intake out when pressed, forward and intake stop when released
		if (gamepad2.b && !b2ButtonPressed) {
			scheduler.schedule(transfer.IntakeDoorIn());
			scheduler.schedule(intake.Out());
			b2ButtonPressed = true;
		} else if (!gamepad2.b && b2ButtonPressed) {
			scheduler.schedule(transfer.IntakeDoorStop());
			scheduler.schedule(intake.Stop());
			b2ButtonPressed = false;
		}

//		if (gamepad2.right_bumper && !dpadUpPressed) {
//			scheduler.schedule(
//					new RepeatCommand(
//							new SequentialCommandGroup(
//									spindexer.NextTarget(),
//									shooter.WaitForTarget().withTimeout(2500L),
//									shooter.WaitForDrop().withTimeout(1000L),
//									new WaitCommand(300)
//							), () -> !gamepad2.dpad_up));
//			dpadUpPressed = true;
//		} else if (!gamepad2.right_bumper && dpadUpPressed) {
//			dpadUpPressed = false;
//		}

		if (gamepad2.x && !dpadUpPressed) {
			scheduler.schedule(spindexer.NextTarget());
			dpadUpPressed = true;
		} else if (!gamepad2.x && dpadUpPressed) {
			dpadUpPressed = false;
		}

		if (gamepad2.y && !yButtonPressed) {
			transfer.SetAutomaticTransfer(false);
			scheduler.schedule(transfer.TransferOut());
		} else if (!gamepad2.y && yButtonPressed) {
			yButtonPressed = false;
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

		panelsTelemetry.addLine("=== LIMELIGHT ===");
		limelight.Telemetry(panelsTelemetry);

		panelsTelemetry.addLine("=== SHOOTER ===");
		panelsTelemetry.addData("Upper RPM", shooter.upperRPM);
		panelsTelemetry.addData("Lower RPM", shooter.lowerRPM);
		panelsTelemetry.addData("Average RPM", shooter.averageRPM);

		panelsTelemetry.addLine("=== TRANSFER ===");
		panelsTelemetry.addData("Shooter At Target", transfer.reachedAverageTarget);
		panelsTelemetry.addData("Spindexer At Target", transfer.spindexerAtTarget);

		panelsTelemetry.update();
	}
}