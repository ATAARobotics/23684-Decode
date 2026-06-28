package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.LowPassFilter;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.PerpetualCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.BeamBreaker;
import org.firstinspires.ftc.teamcode.Subsystem.Conveyor;
import org.firstinspires.ftc.teamcode.Subsystem.Gate;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Limelight;
import org.firstinspires.ftc.teamcode.Subsystem.RGBIndicator;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;
import org.firstinspires.ftc.teamcode.Subsystem.Transfer;
import org.firstinspires.ftc.teamcode.Utils.Drawing;
import org.firstinspires.ftc.teamcode.Utils.RobotConfig;
import org.firstinspires.ftc.teamcode.Utils.RobotPosition;
import org.firstinspires.ftc.teamcode.Utils.Team;
import org.firstinspires.ftc.teamcode.Utils.TeleOpDrive;

import java.util.function.Supplier;

@Configurable
public abstract class MainTeleOp extends OpMode {
	public double conveyorPower = 1;
	protected Follower follower;
	protected CommandScheduler scheduler;
	protected Shooter shooter;
	protected Intake intake;
	protected Transfer transfer;
	protected Conveyor conveyor;
	protected Limelight limelight;
	protected Gate gate;
	protected BeamBreaker beamBreaker;
	protected RGBIndicator rgbIndicator;
	protected int ballCount = 0;
	protected boolean lastIntakeIn = true;
	protected boolean prespinTriggered = false;
	protected TelemetryManager.TelemetryWrapper panelsTelemetry;

	// Button state tracking to prevent continuous input
	protected boolean leftTriggerPressed = false;
	protected boolean rightTriggerPressed = false;
	protected boolean xButtonPressed = false;
	protected boolean aButtonPressed = false;
	protected boolean g2AButtonPressed = false;
	protected boolean b1ButtonPressed = false;
	protected boolean b2ButtonPressed = false;

	protected boolean breakedfollowing = false;

	// Rumble state tracking
	protected boolean wasShooterAtTarget = false;
	protected boolean wasPathBusy = false;
	protected boolean warnedEndGame = false;

	protected boolean warnedHeadinglock = false;

	protected boolean openGate = false;
	protected boolean wasBeamAtThree = false;

	ElapsedTime timer = new ElapsedTime();

	// Drive-input cache. Joystick values drift by ~0.001 per loop even when the
	// driver is holding still. Each call to TeleopDrive writes 4 hardware power
	// values; skip the call entirely when the (x, y, h) inputs haven't changed
	// beyond DRIVE_SNAP_THRESHOLD to cut a hardware-write cluster per loop.
	private static final double DRIVE_SNAP_THRESHOLD = 0.005;
	private double prevDriveX = Double.NaN;
	private double prevDriveY = Double.NaN;
	private double prevDriveH = Double.NaN;

	private Supplier<PathChain> pathBackBlue;
	private Supplier<PathChain> pathFrontBlue;
	private Supplier<PathChain> redGoalShootingPath;
	private Supplier<PathChain> redAudienceShootingPath;

	double upperShooterSpeed = Shooter.AUDIENCE_RPM_UPPER;
	double lowerShooterSpeed = Shooter.AUDIENCE_RPM_LOWER;
	PIDFController headingPIDController;
	TeleOpDrive teleOpDrive;

	double heading;

	boolean headingLock = true;
	double currentHeading, headingdeadzone;

	double targetHeading = Math.toRadians(180);
	double headingCorrection;

	double HumanplayerHeading = 0;
	double Goalx = 0;

//	private double Goalx(Team team){
//		if(team == Team.BLUE){
//			return 0;
//		}else if (team == Team.RED){
//			return 144;
//		}
//		return 0;
//	}

	public static double P = 0.0275, I, D = 0.00025, F = 0.001;

	@Override
	public void init() {
		// Initialize hardware
		follower = Constants.createFollower(hardwareMap);
		if (RobotPosition.isPoseSet) {
			follower.setStartingPose(RobotPosition.robotPose);
//			RobotPosition.isPoseSet = false;
		} else {
			follower.setStartingPose(getStartingPose());
		}
		scheduler = CommandScheduler.getInstance();
		scheduler.reset();
		scheduler.setBulkReading(hardwareMap, LynxModule.BulkCachingMode.AUTO);
		shooter = new Shooter(hardwareMap);
		//scheduler.schedule(shooter.SetTarget(0, 0));
		teleOpDrive = new TeleOpDrive(hardwareMap);

		intake = new Intake(hardwareMap);
		transfer = new Transfer(hardwareMap);
		conveyor = new Conveyor(hardwareMap);
		beamBreaker = new BeamBreaker(hardwareMap);
		gate = new Gate(hardwareMap);
		limelight = new Limelight(hardwareMap, follower);
		// Set shooter dependency for conditional transfer
		transfer.setShooter(shooter);
		rgbIndicator = new RGBIndicator(hardwareMap);
		rgbIndicator.setTransfer(transfer);
		rgbIndicator.setBeamBreaker(beamBreaker);
		rgbIndicator.setFollower(follower);
		// Re-fetch scheduler: scheduler.reset() above nulled the singleton, so
		// the subsystems constructed below registered on a fresh instance. Repoint
		// the local reference at that instance so scheduler.run() invokes their
		// periodic() methods.
		scheduler = CommandScheduler.getInstance();
		//distanceSensor = hardwareMap.get(DistanceSensor.class, "intakeDistanceSensor");

		panelsTelemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();
		headingPIDController = new PIDFController(new PIDFCoefficients(P, I, D, F));

		if (!RobotConfig.COMPETITION) {
			telemetry.addData("Status", "Initialized - Waiting for START");
			telemetry.update();
		}


		pathBackBlue = () -> follower.pathBuilder()
				.addPath(new Path(new BezierLine(follower::getPose, new Pose(91.32791353961615, 17.417498010350027))))
				.setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(-56.53501255275879), 0.8))
				.setNoDeceleration()
				.build();

		pathFrontBlue = () -> follower.pathBuilder()
				.addPath(new Path(new BezierLine(follower::getPose, new Pose(120.4, 95.7))))
				.setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(340.7), 0.8))
				.setNoDeceleration()
				.build();

		redGoalShootingPath = () -> follower.pathBuilder()
				.addPath(new Path(new BezierLine(follower::getPose, new Pose(39.03, 101.80))))
				.setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(203), 0.8))
				.setNoDeceleration()
				.build();

		redAudienceShootingPath = () -> follower.pathBuilder()
				.addPath(new Path(new BezierLine(follower::getPose, new Pose(51.63920244832678,  22.730028047336372))))
				.setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(-122.85147071400115), 0.8))
				.setNoDeceleration()
				.build();
	}

	@Override
	public void start() {
		// Called when START is pressed
		timer.reset();
		timer.startTime();
		scheduler.schedule(gate.closeGate());
		scheduler.run();
	}

	@Override
	public void loop() {
		boolean shooterTriggered = gamepad2.right_trigger > 0.5;
		rgbIndicator.setShooterTriggered(shooterTriggered);

		// CRITICAL - Must complete quickly for responsive driving
		scheduler.run();
		follower.update();
		handleDriveInput();
		handleOperatorInput();
		handleRumbleFeedback();

		if (shooterTriggered) {
			if (gate.getCurrentPosition() != Gate.OPEN_POSITION) {
				scheduler.schedule(gate.openGate());
			}
		} else {
			scheduler.schedule(gate.closeGate());
		}

		// --- Beam Breaker Update ---
		// Determine intake direction from motor power
		double intakePower = intake.getPower();
		Boolean intakeRunningIn;
		if (intakePower > 0) {
			intakeRunningIn = true;
			lastIntakeIn = true;
		} else if (intakePower < 0) {
			intakeRunningIn = false;
			lastIntakeIn = false;
		} else {
			intakeRunningIn = null;
		}

		// Update beam breaker with direction awareness; null indicates stopped
		beamBreaker.update(intakeRunningIn);
		ballCount = beamBreaker.getBallCount();

		// Prespin when ball count reaches 3 (one-shot until count drops)
		if (ballCount >= 3 && !prespinTriggered) {
			prespinTriggered = true;
		} else if (ballCount < 3 && prespinTriggered) {
			prespinTriggered = false;
		}

		displayTelemetry();

		if (gamepad1.xWasPressed()) {
			gamepad1.rumble(300);

			if (getTeam().equals(Team.BLUE)) {
				follower.setPose(new Pose(142.7202744371309, 7.36770930252676, 0));
			} else if (getTeam().equals(Team.RED)) {
				follower.setPose(new Pose(7.1, 18, Math.toRadians(180)));
			}
		}
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
	 * Handle driving input from gamepad1
	 */
	private void handleDriveInput() {
		if (follower.getPose().getY() >= 72.0) {
			upperShooterSpeed = Shooter.GOAL_RPM_UPPER;
			lowerShooterSpeed = Shooter.GOAL_RPM_LOWER;
		} else {
			upperShooterSpeed = Shooter.AUDIENCE_RPM_UPPER;
			lowerShooterSpeed = Shooter.AUDIENCE_RPM_LOWER;
		}


		if (gamepad1.a && !aButtonPressed) {
			breakedfollowing = false;
			if (getTeam() == Team.RED) {
				follower.followPath(redAudienceShootingPath.get(), true);
			} else if (getTeam() == Team.BLUE) {
				follower.followPath(pathBackBlue.get(), true);
			}

			aButtonPressed = true;
		} else if (!gamepad1.a && aButtonPressed) {
			aButtonPressed = false;
		} else if (gamepad1.b && !b1ButtonPressed) {
			breakedfollowing = false;
			if (getTeam() == Team.RED) {
				follower.followPath(redGoalShootingPath.get(), true);
			} else if (getTeam() == Team.BLUE) {
				follower.followPath(pathFrontBlue.get(), true);
			}

			b1ButtonPressed = true;
		} else if (!gamepad1.b && b1ButtonPressed) {
			b1ButtonPressed = false;
		}

		if (!gamepad1.b && !gamepad1.a) {

//			if (!follower.isTeleopDrive()) {
//				follower.startTeleOpDrive(true);
//			}

			if(!breakedfollowing){
				follower.breakFollowing();
				breakedfollowing = true;
			}



			if(getTeam() == Team.RED){
				HumanplayerHeading = Math.toRadians(0);
				Goalx = 144;
			} else if (getTeam() == Team.BLUE) {
				HumanplayerHeading = Math.toRadians(180);
				Goalx = 0;
			}

			if(gamepad1.left_trigger > 0){
				currentHeading = follower.getHeading();
				targetHeading = HumanplayerHeading;
				headingPIDController.setCoefficients(Constants.followerConstants.coefficientsHeadingPIDF);
				headingPIDController.updatePosition(-currentHeading);
				headingdeadzone = 3;
			}else {

				if (limelight.goalsFound(getTeam())) {
					currentHeading = limelight.AngleFrom(getTeam());
					targetHeading = 0;
					headingPIDController.setCoefficients(new PIDFCoefficients(P, I, D, F));
					headingPIDController.updatePosition(-currentHeading);
					headingdeadzone = 1;


				} else {
					currentHeading = follower.getHeading();
					targetHeading = limelight.calculateShotAngle(follower.getPose().getX(),follower.getPose().getY(),Goalx,144);
					headingPIDController.setCoefficients(Constants.followerConstants.coefficientsHeadingPIDF);
					headingPIDController.updatePosition(-currentHeading);
					headingdeadzone = 3;
				}
			}


			headingLock = gamepad1.right_trigger > 0;

			if (headingLock || gamepad1.left_trigger > 0) {


				double headingError = targetHeading - currentHeading;
				headingError = Math.IEEEremainder(headingError, 2 * Math.PI);

				if (Math.abs(headingError) < Math.toRadians(headingdeadzone)) {
					headingCorrection = 0;
					if(warnedHeadinglock) {
						gamepad2.rumble(300);
						warnedHeadinglock = false;
					}
				} else {
					headingPIDController.setTargetPosition(targetHeading);
					headingCorrection = headingPIDController.run();
				}

				//follower.setTeleOpDrive(-gamepad1.left_stick_y,-gamepad1.left_stick_x, -headingCorrection,true);
				writeDriveIfChanged(gamepad1.left_stick_x, gamepad1.left_stick_y, headingCorrection);


			} else {
				warnedHeadinglock = false;
				//follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
				writeDriveIfChanged(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
			}
		}
	}

	/**
	 * Writes driver power to the drivetrain only when the input snapshot has
	 * changed beyond {@link #DRIVE_SNAP_THRESHOLD}. Avoids four redundant
	 * motor-power hardware writes per loop while the driver is holding steady.
	 */
	private void writeDriveIfChanged(double x, double y, double h) {
		if (!Double.isNaN(prevDriveX)
				&& Math.abs(x - prevDriveX) < DRIVE_SNAP_THRESHOLD
				&& Math.abs(y - prevDriveY) < DRIVE_SNAP_THRESHOLD
				&& Math.abs(h - prevDriveH) < DRIVE_SNAP_THRESHOLD) {
			return;
		}
		teleOpDrive.TeleopDrive(follower, x, y, h);
		prevDriveX = x;
		prevDriveY = y;
		prevDriveH = h;
	}

	/**
	 * Handle operator controls from gamepad2
	 */
	protected void handleOperatorInput() {
		// Left Trigger: run intake
		if (gamepad2.left_trigger > 0.5 && !leftTriggerPressed) {
			scheduler.schedule(intake.In());
			scheduler.schedule(transfer.IntakeDoorOut());
			scheduler.schedule(conveyor.In());
			leftTriggerPressed = true;
		} else if (gamepad2.left_trigger <= 0.5 && leftTriggerPressed) {
			scheduler.schedule(intake.Stop());
			scheduler.schedule(transfer.IntakeDoorStop());
			scheduler.schedule(conveyor.Stop());
			leftTriggerPressed = false;
		}

		// Right Trigger: Shooter with conditional transfer (only when trigger held)
		if (gamepad2.right_trigger > 0.5 && !rightTriggerPressed) {
			scheduler.schedule(
					new SequentialCommandGroup(
							shooter.SetTarget(upperShooterSpeed, lowerShooterSpeed),
							new WaitUntilCommand(() -> shooter.getPercentToTarget() >= 0.8),
							shooter.WaitForTarget().withTimeout(2500),
							transfer.TransferOut(),
							conveyor.In()
					));
			rightTriggerPressed = true;
		} else if (gamepad2.right_trigger <= 0.5 && rightTriggerPressed) {
			scheduler.schedule(shooter.SetTarget(0, 0));
			scheduler.schedule(transfer.TransferStop());
			scheduler.schedule(conveyor.Stop());
			if (ballCount > 0) {
				beamBreaker.resetBallCount();
				ballCount = 0;
				prespinTriggered = false;
				wasBeamAtThree = false;
			}
			rightTriggerPressed = false;
		}

		// A Button: Override - reset beam breaker ball count
		if (gamepad2.a && !g2AButtonPressed) {
			beamBreaker.resetBallCount();
			ballCount = 0;
			prespinTriggered = false;
			wasBeamAtThree = false;
			g2AButtonPressed = true;
		} else if (!gamepad2.a && g2AButtonPressed) {
			g2AButtonPressed = false;
		}

		if (gamepad2.yWasPressed() || ballCount >= 3) {
			scheduler.schedule(shooter.SetTarget(upperShooterSpeed, lowerShooterSpeed));
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
			scheduler.schedule(conveyor.Out());
			b2ButtonPressed = true;
		} else if (!gamepad2.b && b2ButtonPressed) {
			scheduler.schedule(transfer.IntakeDoorStop());
			scheduler.schedule(intake.Stop());
			scheduler.schedule(conveyor.Stop());
			b2ButtonPressed = false;
		}

		// Right bumper: Adjust conveyor power
		if (gamepad2.right_bumper) {
			conveyorPower = 0.6;
		} else {
			conveyorPower = 1;
		}

		// Left joystick: Conveyor control proportional to joystick movement (inverted Y axis)
		double leftJoystickY = -gamepad2.left_stick_y;

		// Dead zone: stop conveyor
		if (Math.abs(gamepad2.left_stick_y) < 0.2) {
			scheduler.schedule(conveyor.Stop());
			scheduler.schedule(transfer.IntakeDoorStop());
		} else {
			// Proportional control: power is proportional to joystick position
			scheduler.schedule(new PerpetualCommand(conveyor.DirectPower(leftJoystickY * conveyorPower)));

			if (leftJoystickY > 0) {
				scheduler.schedule(transfer.IntakeDoorOut());
			} else {
				scheduler.schedule(transfer.IntakeDoorIn());
			}
		}
	}

	/**
	 * Display telemetry information
	 */
	protected void displayTelemetry() {
		if (RobotConfig.COMPETITION) return;

		panelsTelemetry.addData("Location", follower.getPose().toString());
		Drawing.drawRobot(follower.getPose());

		panelsTelemetry.addData("Shooter At Target", transfer.reachedAverageTarget);

		panelsTelemetry.addData("Shooter Lower RPM", shooter.lowerRPM);
		panelsTelemetry.addData("Shooter Upper RPM", shooter.upperRPM);
		panelsTelemetry.addData("Shooter Lower Target", shooter.lowerTarget);
		panelsTelemetry.addData("Shooter Upper Target", shooter.upperTarget);

		beamBreaker.telemetry(panelsTelemetry);

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

		wasShooterAtTarget = transfer.reachedAverageTarget;

		// Driver 2: Rumble when beam breaker first reaches 3 artifacts
		if (ballCount >= 3 && !wasBeamAtThree) {
			gamepad2.rumble(250);
		}
		wasBeamAtThree = ballCount >= 3;
	}
}