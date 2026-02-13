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
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.PerpetualCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.UninterruptibleCommand;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Limelight;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;
import org.firstinspires.ftc.teamcode.Subsystem.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystem.Transfer;
import org.firstinspires.ftc.teamcode.Utils.RobotPosition;
import org.firstinspires.ftc.teamcode.Utils.Team;

import java.util.function.Supplier;

@Config
@Configurable
public abstract class MainTeleOp extends OpMode {
	public  double spindexerPower = 1;
	protected Follower follower;
	protected CommandScheduler scheduler;
	protected Shooter shooter;
	protected Intake intake;
	protected Transfer transfer;
	protected Spindexer spindexer;
	//protected Limelight limelight;
	protected TelemetryManager.TelemetryWrapper panelsTelemetry;
	// Button state tracking to prevent continuous input
	protected boolean leftTriggerPressed = false;
	protected boolean rightTriggerPressed = false;
	protected boolean xButtonPressed = false;
	protected boolean aButtonPressed = false;
	protected boolean g2AButtonPressed = false;
	protected boolean b1ButtonPressed = false;
	protected boolean b2ButtonPressed = false;
	protected boolean dpadUpPressed = false;
	protected boolean dpadDownPressed = false;
	protected boolean yButtonPressed = false;
	protected boolean spindexerUpCrossed = false;
	protected boolean spindexerMidCrossed = false;
	protected boolean spindexerDownCrossed = false;
	// Rumble state tracking
	protected boolean wasShooterAtTarget = false;
	protected boolean wasPathBusy = false;

	boolean ArtifactFound = false;
	boolean ToggleDistance = false;

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
	private Supplier<PathChain> redGoalShootingPath;
	private Supplier<PathChain> redAudienceShootingPath;

	double upperShooterSpeed = Shooter.AUDIENCE_RPM;
	double lowerShooterSpeed = Shooter.AUDIENCE_RPM;
	DistanceSensor distanceSensor;

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
		//light = new Limelight(hardwareMap);
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
		distanceSensor = hardwareMap.get(DistanceSensor.class, "intakeDistanceSensor");
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
				.addPath(new Path(new BezierLine(follower::getPose, new Pose(79, 96.361))))
				.setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading,Math.toRadians(330), 0.8))
				.build();

		redGoalShootingPath = () -> follower.pathBuilder()
				.addPath(new Path(new BezierLine(follower::getPose, new Pose(63, 96.361))))
				.setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading,Math.toRadians(210), 0.8))
				.build();

		redAudienceShootingPath = () -> follower.pathBuilder()
				.addPath(new Path(new BezierLine(follower::getPose, new Pose(79, 11))))
				.setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(245), 0.8))
				.build();
	}

	@Override
	public void init_loop() {

	}

	@Override
	public void start() {
		// Called when START is pressed
//		limelight.start();
		spindexer.zeroSpindexer();
		timer.startTime();
		//scheduler.run();
	}

	@Override
	public void loop() {
		long startTime = System.nanoTime();

		// CRITICAL - Must complete quickly for responsive driving
		follower.update();
		handleDriveInput();
		handleOperatorInput();
		updateRGBIndicator();
		handleRumbleFeedback();
		scheduler.run();
		shooter.periodic();
		spindexer.periodic();

		displayTelemetry();
		// Performance monitoring
		long loopTime = System.nanoTime() - startTime;

		if (loopTime > maxLoopTime) {
			maxLoopTime = loopTime;
		}

		//limelight.updateLowPass(follower.getHeading());
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

			if(getTeam().equals(Team.BLUE)){
			follower.setPose(new Pose(142.7202744371309, 7.36770930252676, 0));
			}
			else if (getTeam().equals(Team.RED)){
				follower.setPose(new Pose(6.2797255628690891,7.36770930252676, Math.toRadians(180)));
			}
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

		if(follower.getPose().getY() >= 72.0) {
			upperShooterSpeed = Shooter.GOAL_RPM_UPPER;
			lowerShooterSpeed = Shooter.GOAL_RPM_LOWER;
		}else{
			upperShooterSpeed = Shooter.AUDIENCE_RPM;
			lowerShooterSpeed = Shooter.AUDIENCE_RPM;
		}

		if (gamepad1.cross && !aButtonPressed) {

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
			if (getTeam() == Team.RED) {
				follower.followPath(redGoalShootingPath.get(), true);
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
			shooter.setTarget(upperShooterSpeed, lowerShooterSpeed);
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
		if (gamepad2.b && !b2ButtonPressed) {
			scheduler.schedule(transfer.IntakeDoorIn());
			scheduler.schedule(intake.Out());
			b2ButtonPressed = true;
		} else if (!gamepad2.b && b2ButtonPressed) {
			scheduler.schedule(transfer.IntakeDoorStop());
			scheduler.schedule(intake.Stop());
			b2ButtonPressed = false;
		}

		// Dpad Up: Run spindexer while held, go to next target on release
		if (gamepad2.dpad_up && !dpadUpPressed) {
			scheduler.schedule(spindexer.NextTarget());
			scheduler.schedule(transfer.IntakeDoorOut());
			dpadUpPressed = true;
		} else if (!gamepad2.dpad_up && dpadUpPressed) {
			scheduler.schedule(transfer.IntakeDoorStop());
			dpadUpPressed = false;
			spindexerMidCrossed = true;
			spindexerUpCrossed = false;
			spindexerDownCrossed = false;
		}

		if(gamepad2.right_bumper){
			spindexerPower = 0.5;
		}else{
			spindexerPower = 1;
		}


		if (gamepad2.dpad_down && !dpadDownPressed) {
			scheduler.schedule(spindexer.DirectPower(spindexerPower));
			scheduler.schedule(transfer.IntakeDoorOut());
			dpadUpPressed = true;
		} else if (!gamepad2.dpad_down && dpadDownPressed) {
			scheduler.schedule(spindexer.NextTarget());
			scheduler.schedule(transfer.IntakeDoorStop());
			dpadDownPressed = false;
			spindexerMidCrossed = true;
			spindexerUpCrossed = false;
			spindexerDownCrossed = false;
		}

		// A Button: Reset spindexer position
		if (gamepad2.a && !g2AButtonPressed) {
			scheduler.schedule(new InstantCommand(spindexer::zeroSpindexer));
			g2AButtonPressed = true;
		} else if (!gamepad2.a && g2AButtonPressed) {
			g2AButtonPressed = false;
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

//		if(Math.abs(gamepad2.left_stick_y) < 0.1 && !gamepad2.dpad_up){
//			ToggleDistance = distanceSensor.getDistance(DistanceUnit.CM) < 12;
//
//			if(ToggleDistance && !ArtifactFound){
//				scheduler.schedule(new SequentialCommandGroup(
//						new ParallelCommandGroup(spindexer.DirectPower(1),new WaitCommand(270)),
//						spindexer.DirectPower(0)));
//				ArtifactFound = true;
//			}if(!ToggleDistance && ArtifactFound){
//				//scheduler.schedule(spindexer.DirectPower(0));
//				ArtifactFound = false;
//			}
//
//		}

		// Left joystick: Spindexer control proportional to joystick movement (inverted Y axis)
		double leftJoystickY = -gamepad2.left_stick_y;

		// Dead zone: stop spindexer
		if (Math.abs(gamepad2.left_stick_y) < 0.2) {
			leftJoystickY = 0;
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
		} else {
			// Proportional control: power is proportional to joystick position
			scheduler.schedule(new PerpetualCommand(spindexer.DirectPower(leftJoystickY * spindexerPower)));

			if (leftJoystickY > 0) {
				scheduler.schedule(transfer.IntakeDoorOut());
				if (!gamepad2.x) {
					scheduler.schedule(transfer.TransferIn());
				}
			} else {
				scheduler.schedule(transfer.IntakeDoorIn());
				if (!gamepad2.x) {
					scheduler.schedule(transfer.TransferIn());
				}
			}

			spindexerMidCrossed = false;
			spindexerUpCrossed = false;
			spindexerDownCrossed = false;
		}
	}

	/**
	 * Display telemetry information
	 */
	protected void displayTelemetry() {
		panelsTelemetry.addLine("=== MAIN TELEOP ===");
		panelsTelemetry.addData("Drive Mode", "Mecanum");
		panelsTelemetry.addData("Location", follower.getPose().toString());

//		panelsTelemetry.addLine("=== LIMELIGHT ===");
//		limelight.Telemetry(panelsTelemetry);

		panelsTelemetry.addLine("=== SHOOTER ===");
		panelsTelemetry.addData("Upper RPM", shooter.upperRPM);
		panelsTelemetry.addData("Lower RPM", shooter.lowerRPM);
		panelsTelemetry.addData("Average RPM", shooter.averageRPM);

		panelsTelemetry.addLine("=== TRANSFER ===");
		panelsTelemetry.addData("Shooter At Target", transfer.reachedAverageTarget);
		panelsTelemetry.addData("Spindexer At Target", transfer.spindexerAtTarget);

		panelsTelemetry.update();
	}

	/**
	 * Handle controller rumble feedback
	 */
	private void handleRumbleFeedback() {
		// Driver 1: Rumble when path following (A button) is complete
		boolean isPathBusy = follower.isBusy();
		if (gamepad1.a && wasPathBusy && !isPathBusy) {
			gamepad1.rumble(500);
		}
		wasPathBusy = isPathBusy;

		// Driver 2: Rumble when shooter reaches target RPM
		if (transfer.reachedAverageTarget && !wasShooterAtTarget) {
			gamepad2.rumble(500);
		}
		wasShooterAtTarget = transfer.reachedAverageTarget;
	}
}