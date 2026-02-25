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
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.PerpetualCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;
import org.firstinspires.ftc.teamcode.Subsystem.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystem.Transfer;
import org.firstinspires.ftc.teamcode.Utils.RobotPosition;
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
//	protected Colour colour;
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
		scheduler.schedule(shooter.SetTarget(0, 0));
		intake = new Intake(hardwareMap);
		transfer = new Transfer(hardwareMap);
		spindexer = new Spindexer(hardwareMap);
//		colour = new Colour(hardwareMap);
		// Set shooter dependency for conditional transfer
		transfer.setShooter(shooter);
		//transfer.setSpindexer(spindexer);
		// TODO: Make subsystem
		rgbServo = hardwareMap.get(Servo.class, "rgbIndicator");
		distanceSensor = hardwareMap.get(DistanceSensor.class, "intakeDistanceSensor");
		intakeTouchSensor = hardwareMap.get(TouchSensor.class, "intakeTouchSensor");


		panelsTelemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();

		telemetry.addData("Status", "Initialized - Waiting for START");
		telemetry.update();

		pathBackBlue = () -> follower.pathBuilder()
				.addPath(new Path(new BezierLine(follower::getPose, new Pose(65, 11))))
				.setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(295), 0.8))
				.build();

		pathFrontBlue = () -> follower.pathBuilder()
				.addPath(new Path(new BezierLine(follower::getPose, new Pose(107, 101))))
				.setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading,Math.toRadians(344), 0.8))
				.build();

		redGoalShootingPath = () -> follower.pathBuilder()
				.addPath(new Path(new BezierLine(follower::getPose, new Pose(39.03, 101.80))))
				.setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading,Math.toRadians(203), 0.8))
				.build();

		redAudienceShootingPath = () -> follower.pathBuilder()
				.addPath(new Path(new BezierLine(follower::getPose, new Pose(79, 11))))
				.setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(245), 0.8))
				.build();
	}

	@Override
	public void start() {
		// Called when START is pressed
		spindexer.zeroSpindexer();
		timer.startTime();
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
		spindexer.periodic();
		// TODO: Call colour.update() with the current spindexer slot once getCurrentSlot() is implemented
		 //colour.update(spindexer.getCurrentSlot());

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
				follower.setPose(new Pose(5, 7, Math.toRadians(180)));
			}
//			follower.setPose(new Pose(136.039, 78.907317073, 0)); testing only
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
			leftTriggerPressed = true;
		} else if (gamepad2.left_trigger <= 0.5 && leftTriggerPressed) {
			scheduler.schedule(intake.Stop());
			scheduler.schedule(transfer.IntakeDoorStop());
			leftTriggerPressed = false;
		}

		boolean lockTransfer = false;

		// Right Trigger: Shooter with conditional transfer (only when trigger held)
		if (gamepad2.right_trigger > 0.5 && !rightTriggerPressed) {
			shooter.setTarget(upperShooterSpeed, lowerShooterSpeed);
			scheduler.schedule(new SequentialCommandGroup(
					shooter.WaitForTarget(),
					spindexer.DirectPower(0.3)
			));
			rightTriggerPressed = true;
			yButtonPressed = false;
		} else if (gamepad2.right_trigger <= 0.5 && rightTriggerPressed) {
			shooter.setTarget(0, 0);
			scheduler.schedule(transfer.TransferStop());
			scheduler.schedule(spindexer.DirectPower(0));
			rightTriggerPressed = false;
		}


		if (gamepad2.y && !yButtonPressed) {
			shooter.setTarget(upperShooterSpeed, lowerShooterSpeed);
			yButtonPressed = true;
		} else if (!gamepad2.y && yButtonPressed) {
			//shooter.setTarget(0, 0);
			//scheduler.schedule(transfer.TransferStop());
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
			if ((Math.abs(gamepad2.left_stick_y) > 0.2 || rightTriggerPressed) && yButtonPressed) {
				transfer.runAutomaticTransfer = true;
				transfer.updateAutomaticTransfer(false);
			} else if ((Math.abs(gamepad2.left_stick_y) < 0.2 || !rightTriggerPressed) && !yButtonPressed) {
				transfer.runAutomaticTransfer = false;
				scheduler.schedule(transfer.TransferIn());
			} else{
				transfer.runAutomaticTransfer = false;
				scheduler.schedule(transfer.TransferStop());
			}
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

		// Right bumper: Adjust spindexer power
		if (gamepad2.right_bumper) {
			spindexerPower = 0.1;
		} else {
			spindexerPower = 0.3;
		}

		// Dpad Down: Manual spindexer control
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
//			eee
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
//				if (!gamepad2.x || !rightTriggerPressed) {
//					scheduler.schedule(transfer.SetAutomaticTransfer(false));
//				}
				spindexerMidCrossed = true;
				spindexerUpCrossed = false;
				spindexerDownCrossed = false;
			}
		} else {
			// Proportional control: power is proportional to joystick position
			scheduler.schedule(new PerpetualCommand(spindexer.DirectPower(leftJoystickY * spindexerPower)));

			if (leftJoystickY > 0) {
				scheduler.schedule(transfer.IntakeDoorOut());
//				if (!gamepad2.x || !rightTriggerPressed) {
//					scheduler.schedule(transfer.SetAutomaticTransfer(true));
//				}
			} else {
				scheduler.schedule(transfer.IntakeDoorIn());
//				if (!gamepad2.x || !rightTriggerPressed) {
//					scheduler.schedule(transfer.SetAutomaticTransfer(true));
//				}
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

		panelsTelemetry.addLine("=== SHOOTER ===");
		panelsTelemetry.addData("Upper RPM", shooter.upperRPM);
		panelsTelemetry.addData("Lower RPM", shooter.lowerRPM);
		panelsTelemetry.addData("Average RPM", shooter.averageRPM);
		panelsTelemetry.addData("Amps Drawn to Shooter", shooter.TotalCurrentDrawn());

		panelsTelemetry.addLine("=== TRANSFER ===");
		panelsTelemetry.addData("Shooter At Target", transfer.reachedAverageTarget);
		panelsTelemetry.addData("Spindexer At Target", transfer.spindexerAtTarget);

		panelsTelemetry.addLine("=== COLOUR ===");
//		panelsTelemetry.addData("Slot 1", colour.colours.getSlot1().toString());
//		panelsTelemetry.addData("Slot 2", colour.colours.getSlot2().toString());
//		panelsTelemetry.addData("Slot 3", colour.colours.getSlot3().toString());
		// TODO: Add current slot telemetry once getCurrentSlot() is implemented

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

//		if(intakeTouchSensor.isPressed()){
//			gamepad2.rumble(500);
//		}
		wasShooterAtTarget = transfer.reachedAverageTarget;
	}
}