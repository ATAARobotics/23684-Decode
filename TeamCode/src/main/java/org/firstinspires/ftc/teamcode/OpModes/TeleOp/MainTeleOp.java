package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

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
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.PerpetualCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.Gate;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.NewShooter;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;
import org.firstinspires.ftc.teamcode.Subsystem.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystem.Transfer;
import org.firstinspires.ftc.teamcode.Subsystem.Colour;
import org.firstinspires.ftc.teamcode.Utils.Drawing;
import org.firstinspires.ftc.teamcode.Utils.RobotPosition;
import org.firstinspires.ftc.teamcode.Utils.ShootArtifacts;
import org.firstinspires.ftc.teamcode.Utils.Team;

import java.util.function.Supplier;

@Configurable
public abstract class MainTeleOp extends OpMode {
	public  double spindexerPower = 1;
	protected Follower follower;
	protected CommandScheduler scheduler;
	protected Shooter shooter;
	protected Intake intake;
	protected Transfer transfer;
	protected Spindexer spindexer;
	protected NewShooter newShooter;
	protected Gate gate;
	protected Colour colour;
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
	protected boolean warnedEndGame = false;

	protected boolean openGate = false;

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
	//DistanceSensor distanceSensor;
	TouchSensor intakeTouchSensor;

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
		scheduler.setBulkReading(hardwareMap, LynxModule.BulkCachingMode.AUTO);
		shooter = new Shooter(hardwareMap);
		//scheduler.schedule(shooter.SetTarget(0, 0));

		intake = new Intake(hardwareMap);
		transfer = new Transfer(hardwareMap);
		spindexer = new Spindexer(hardwareMap);
		newShooter = new NewShooter(hardwareMap);
		scheduler.schedule(newShooter.setTarget(0,0));
		gate = new Gate(hardwareMap);
		spindexer.zeroSpindexer();
		if (RobotPosition.isSpindexerSet) {
			RobotPosition.isSpindexerSet = false;
		}
		colour = new Colour(hardwareMap);
		// Set shooter dependency for conditional transfer
		transfer.setShooter(shooter);
		//transfer.setSpindexer(spindexer);
		// TODO: Make subsystem
		rgbServo = hardwareMap.get(Servo.class, "rgbIndicator");
		//distanceSensor = hardwareMap.get(DistanceSensor.class, "intakeDistanceSensor");
		intakeTouchSensor = hardwareMap.get(TouchSensor.class, "intakeTouchSensorLeft");

		panelsTelemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();

		telemetry.addData("Status", "Initialized - Waiting for START");
		telemetry.update();

		pathBackBlue = () -> follower.pathBuilder()
				.addPath(new Path(new BezierLine(follower::getPose, new Pose(65, 11))))
				.setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(295), 0.8))
				.build();

		pathFrontBlue = () -> follower.pathBuilder()
				.addPath(new Path(new BezierLine(follower::getPose, new Pose(120.4, 95.7))))
				.setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading,Math.toRadians(340.7), 0.8))
				.build();

		redGoalShootingPath = () -> follower.pathBuilder()
				.addPath(new Path(new BezierLine(follower::getPose, new Pose(39.03, 101.80))))
				.setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading,Math.toRadians(203), 0.8))
				.build();

		redAudienceShootingPath = () -> follower.pathBuilder()
				.addPath(new Path(new BezierLine(follower::getPose, new Pose(79, 15))))
				.setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(247.5), 0.8))
				.build();
	}

	@Override
	public void start() {
		// Called when START is pressed
		// Spindexer already initializes from RobotPosition in constructor
		timer.reset();
		timer.startTime();
		scheduler.schedule(gate.closeGate());
		scheduler.run();
	}

	@Override
	public void loop() {
		long startTime = System.nanoTime();

		// CRITICAL - Must complete quickly for responsive driving
		scheduler.run();
		follower.update();
		handleDriveInput();
		handleOperatorInput();
		updateRGBIndicator();
		handleRumbleFeedback();
		shooter.periodic();

		if (openGate){scheduler.schedule(gate.openGate());}
		else{scheduler.schedule(gate.closeGate());}

		int currentSlot = spindexer.getCurrentSlot();
		if (currentSlot != -1) {
			colour.update(currentSlot);
		}

		// Clear slot when transfer is running forward
		if (transfer.transferLeft.getPower() > 0.5) {
			colour.clearCurrentSlot(currentSlot);
		}

		displayTelemetry();
		// Performance monitoring
		long loopTime = System.nanoTime() - startTime;

		if (loopTime > maxLoopTime) {
			maxLoopTime = loopTime;
		}

		if (gamepad1.xWasPressed()) {
			gamepad1.rumble(300);

			if (getTeam().equals(Team.BLUE)) {
				follower.setPose(new Pose(142.7202744371309, 7.36770930252676, 0));
			} else if (getTeam().equals(Team.RED)) {
				follower.setPose(new Pose(7.1, 18, Math.toRadians(180)));
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
		// Ensure we don't reuse values if TeleOp is accidentally stopped and restarted
		RobotPosition.isSpindexerSet = false;
		RobotPosition.spindexerTicks = 0;
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
			rgbServo.setPosition(transfer.reachedUpperTarget && transfer.reachedLowerTarget ? 0.50 : 0.28);
		} else {
			rgbServo.setPosition(0.65);
		}
	}

	/**
	 * Handle driving input from gamepad1
	 */
	private void handleDriveInput() {
		if ( follower.getPose().getY() >= 72.0) {
			upperShooterSpeed = Shooter.GOAL_RPM_UPPER;
			lowerShooterSpeed = Shooter.GOAL_RPM_LOWER;
		} else{
			upperShooterSpeed = Shooter.AUDIENCE_RPM_UPPER;
			lowerShooterSpeed = Shooter.AUDIENCE_RPM_LOWER;
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
		} else if (gamepad1.circle && !b1ButtonPressed) {
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
			scheduler.schedule(spindexer.DirectPower(spindexerPower));
			leftTriggerPressed = true;
		} else if (gamepad2.left_trigger <= 0.5 && leftTriggerPressed) {
			scheduler.schedule(intake.Stop());
			scheduler.schedule(transfer.IntakeDoorStop());
			scheduler.schedule(spindexer.DirectPower(0));
			leftTriggerPressed = false;
		}

		boolean lockTransfer = false;

		// Right Trigger: Shooter with conditional transfer (only when trigger held)
		if (gamepad2.right_trigger > 0.5 && !rightTriggerPressed) {
			scheduler.schedule(
					new SequentialCommandGroup(
							shooter.SetTarget(Shooter.AUDIENCE_RPM , Shooter.AUDIENCE_RPM),
							new WaitUntilCommand(() -> shooter.getPercentToTarget() >= 0.8),
							new InstantCommand(() -> openGate = true),
							shooter.WaitForTarget().withTimeout(2500),
							transfer.TransferOut(),
							spindexer.DirectPower(1)
					));
			rightTriggerPressed = true;
		} else if (gamepad2.right_trigger <= 0.5 && rightTriggerPressed) {
			scheduler.schedule(shooter.SetTarget(0, 0));
			scheduler.schedule(transfer.TransferStop());
			scheduler.schedule(spindexer.DirectPower(0));
			openGate = false;
			rightTriggerPressed = false;
		}

		if(gamepad2.yWasPressed()){
			shooter.SetTarget(upperShooterSpeed,lowerShooterSpeed);
		}

		// X Button: Override transfer forward - manual control
		if (gamepad2.x && !xButtonPressed) {
			scheduler.schedule(transfer.TransferOut());
			xButtonPressed = true;
		} else if (!gamepad2.x && xButtonPressed && !rightTriggerPressed) {
			scheduler.schedule(transfer.TransferStop());
			xButtonPressed = false;
		}

		// B Button: Intake door backward and intake out when pressed, forward and intake stop when released
		if (gamepad2.b && !b2ButtonPressed) {
			scheduler.schedule(transfer.IntakeDoorIn());
			scheduler.schedule(intake.Out());
			scheduler.schedule(spindexer.DirectPower(-spindexerPower));
			b2ButtonPressed = true;
		} else if (!gamepad2.b && b2ButtonPressed) {
			scheduler.schedule(transfer.IntakeDoorStop());
			scheduler.schedule(intake.Stop());
			scheduler.schedule(spindexer.DirectPower(0));
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

		// Right bumper: Adjust spindexer power
		if (gamepad2.right_bumper) {
			spindexerPower = 0.6;
		} else {
			spindexerPower = 1;
		}

		// Dpad Down: Manual spindexer control
		if (gamepad2.dpad_down && !dpadDownPressed) {
			scheduler.schedule(spindexer.DirectPower(spindexerPower));
			scheduler.schedule(transfer.IntakeDoorOut());
			dpadUpPressed = true;
		} else if (!gamepad2.dpad_down && dpadDownPressed) {
			scheduler.schedule(transfer.IntakeDoorStop());
			dpadDownPressed = false;
			spindexerMidCrossed = true;
			spindexerUpCrossed = false;
			spindexerDownCrossed = false;
		}

		// Left joystick: Spindexer control proportional to joystick movement (inverted Y axis)
		double leftJoystickY = -gamepad2.left_stick_y;

		// Dead zone: stop spindexer
		if (Math.abs(gamepad2.left_stick_y) < 0.2) {
			leftJoystickY = 0;
			if (!spindexerMidCrossed) {
				scheduler.schedule(spindexer.DirectPower(0));
				scheduler.schedule(transfer.IntakeDoorStop());
				spindexerMidCrossed = true;
				spindexerUpCrossed = false;
				spindexerDownCrossed = false;
			}
		} else {
			// Proportional control: power is proportional to joystick position
			scheduler.schedule(new PerpetualCommand(spindexer.DirectPower(leftJoystickY * spindexerPower)));

			if (leftJoystickY > 0) {
				scheduler.schedule(transfer.IntakeDoorOut());
			} else {
				scheduler.schedule(transfer.IntakeDoorIn());
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
		Drawing.drawRobot(follower.getPose());

		panelsTelemetry.addLine("=== SHOOTER ===");
		panelsTelemetry.addData("Upper ticks", newShooter.upperShooter.getVelocity());
		panelsTelemetry.addData("Lower RPM", newShooter.upperShooter.getVelocity());
		panelsTelemetry.addData("Target",NewShooter.AUDIENCE_TPR);
		panelsTelemetry.addData("At Target ?", newShooter.isAtTarget());

		panelsTelemetry.addLine("=== TRANSFER ===");
		panelsTelemetry.addData("Shooter At Target", transfer.reachedAverageTarget);
		panelsTelemetry.addData("Spindexer At Target", transfer.spindexerAtTarget);

		spindexer.Telemetry(panelsTelemetry);

		panelsTelemetry.addLine("=== COLOUR SENSORS ===");
		panelsTelemetry.addData("Update Count", colour.updateCount);
		panelsTelemetry.addData("Is Updating", colour.isUpdating);
		panelsTelemetry.addData("Last Update (ms ago)", System.currentTimeMillis() - colour.lastUpdateTime);

		panelsTelemetry.addLine("--- Slot 2 Sensor ---");
		panelsTelemetry.addData("RGB", "R=%.2f G=%.2f B=%.2f", colour.slot2Red, colour.slot2Green, colour.slot2Blue);
		panelsTelemetry.addData("HSV", "H=%.1f° S=%.2f V=%.2f", colour.slot2Hue, colour.slot2Saturation, colour.slot2Value);
		panelsTelemetry.addData("Detected Colour", colour.colours.getSlot2().toString());

		panelsTelemetry.addLine("--- Slot 3 Sensor ---");
		panelsTelemetry.addData("RGB", "R=%.2f G=%.2f B=%.2f", colour.slot3Red, colour.slot3Green, colour.slot3Blue);
		panelsTelemetry.addData("HSV", "H=%.1f° S=%.2f V=%.2f", colour.slot3Hue, colour.slot3Saturation, colour.slot3Value);
		panelsTelemetry.addData("Detected Colour", colour.colours.getSlot3().toString());

		panelsTelemetry.addLine("--- Mapped Slots ---");
		panelsTelemetry.addData("Slot 1", colour.colours.getSlot1().toString());
		panelsTelemetry.addData("Slot 2", colour.colours.getSlot2().toString());
		panelsTelemetry.addData("Slot 3", colour.colours.getSlot3().toString());

		panelsTelemetry.update();
	}

	/**
	 * Handle controller rumble feedback
	 */
	private void handleRumbleFeedback() {
		// Match Timer Alert: 21 seconds remaining in 2:00 TeleOp (99 seconds elapsed)
		if (timer.seconds() >= 99 && !warnedEndGame) {
			gamepad1.rumbleBlips(3);
			gamepad2.rumbleBlips(3);
			warnedEndGame = true;
		}

		// Driver 1: Rumble when path following (A button) is complete
		boolean isPathBusy = follower.isBusy();
		if (gamepad1.a && wasPathBusy && !isPathBusy) {
			gamepad2.rumble(100);
		}
		wasPathBusy = isPathBusy;

		// Driver 2: Rumble when shooter reaches target RPM
		if (transfer.reachedAverageTarget && !wasShooterAtTarget) {
			gamepad2.rumble(500);
		}

//		if(intakeTouchSensor.isPressed()){
//			gamepad2.rumble(500);
//		}
		wasShooterAtTarget = transfer.reachedAverageTarget;
	}
}