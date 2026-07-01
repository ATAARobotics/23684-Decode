package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
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

import org.firstinspires.ftc.teamcode.OpModes.Auto.Modular.PoseDatabase;
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
	protected boolean leftTriggerPressed = false;
	protected boolean rightTriggerPressed = false;
	protected boolean xButtonPressed = false;
	protected boolean aButtonPressed = false;
	protected boolean gamepad2AWasPressed = false;
	protected boolean gamepad1BWasPressed = false;
	protected boolean gamepad2BWasPressed = false;
	protected boolean leftJoystickDeadzone = true;
	protected boolean brokeFollowing = false;
	protected boolean wasShooterAtTarget = false;
	protected boolean wasPathBusy = false;
	protected boolean warnedEndGame = false;
	protected boolean headingLockRumbleSent = false;
	protected boolean wasBeamAtThree = false;
	ElapsedTime timer = new ElapsedTime();
	// Drive-input cache. Joystick values drift by ~0.001 per loop even when the
	// driver is holding still. Each call to TeleopDrive writes 4 hardware power
	// values; skip the call entirely when (x, y, h) inputs haven't changed
	// beyond DRIVE_SNAP_THRESHOLD, saving a hardware-write cluster per loop.
	private static final double DRIVE_SNAP_THRESHOLD = 0.005;
	private double prevDriveX = Double.NaN;
	private double prevDriveY = Double.NaN;
	private double prevDriveH = Double.NaN;
	private Supplier<PathChain> pathAudienceShoot;
	private Supplier<PathChain> pathGoalShoot;
	double upperShooterSpeed = Shooter.AUDIENCE_RPM_UPPER;
	double lowerShooterSpeed = Shooter.AUDIENCE_RPM_LOWER;
	PIDFController headingPIDController;
	TeleOpDrive teleOpDrive;
	boolean headingLock = true;
	double currentHeading;
	double headingDeadzone;
	double targetHeading = Math.toRadians(180);
	double headingCorrection;
	double humanPlayerHeading = 0;
	double goalX = 0;
	boolean openGate = false;
	public static double P = 0.0275, I, D = 0.00025, F = 0.001;

	@Override
	public void init() {
		follower = Constants.createFollower(hardwareMap);
		if (RobotPosition.isPoseSet) {
			follower.setStartingPose(RobotPosition.robotPose);
			//RobotPosition.isPoseSet = false;
		} else {
			follower.setStartingPose(getStartingPose());
		}

		scheduler = CommandScheduler.getInstance();
		scheduler.reset();
		scheduler.setBulkReading(hardwareMap, LynxModule.BulkCachingMode.AUTO);

		shooter = new Shooter(hardwareMap);
		teleOpDrive = new TeleOpDrive(hardwareMap);

		intake = new Intake(hardwareMap);
		transfer = new Transfer(hardwareMap);
		conveyor = new Conveyor(hardwareMap);
		beamBreaker = new BeamBreaker(hardwareMap);
		gate = new Gate(hardwareMap);
		limelight = new Limelight(hardwareMap, follower);
		transfer.setShooter(shooter);

		rgbIndicator = new RGBIndicator(hardwareMap);
		rgbIndicator.setTransfer(transfer);
		rgbIndicator.setBeamBreaker(beamBreaker);
		rgbIndicator.setFollower(follower);

		// Repoint at the post-reset scheduler singleton so scheduler.run() drives
		// the subsystems registered above on the new instance.
		scheduler = CommandScheduler.getInstance();

		panelsTelemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();
		headingPIDController = new PIDFController(new PIDFCoefficients(P, I, D, F));

		if (!RobotConfig.COMPETITION) {
			telemetry.addData("Status", "Initialized - Waiting for START");
			telemetry.update();
		}

		pathAudienceShoot = () -> buildShootPath(PoseDatabase.getAudienceShootPose(getTeam()), Math.toRadians(-56.535));
		pathGoalShoot = () -> buildShootPath(PoseDatabase.getGoalShootPose(getTeam()), Math.toRadians(340.7));
	}

	private PathChain buildShootPath(Pose endPose, double endHeadingDeg) {
		return follower.pathBuilder()
				.addPath(new Path(new BezierLine(follower::getPose, endPose)))
				.setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, endHeadingDeg, 0.8))
				.setNoDeceleration()
				.build();
	}

	@Override
	public void start() {
		timer.reset();
		timer.startTime();
		scheduler.schedule(gate.closeGate());
		scheduler.run();
		if (RobotPosition.isPoseSet) { //TODO: find out why to robot starts in a random positon without this
			follower.setPose(RobotPosition.robotPose);
			RobotPosition.isPoseSet = false;
		} else {
			follower.setPose(getStartingPose());
		}
	}

	@Override
	public void loop() {
		boolean shooterTriggered = gamepad2.right_trigger > 0.5;
		rgbIndicator.setShooterTriggered(shooterTriggered);

		scheduler.run();
		follower.update();
		handleDriveInput();
		handleOperatorInput();
		handleRumbleFeedback();

		if (openGate) {
			if (gate.getCurrentPosition() != Gate.OPEN_POSITION) {
				scheduler.schedule(gate.openGate());
			}
		} else {
			if (gate.getCurrentPosition() != Gate.CLOSE_POSITION) {
				scheduler.schedule(gate.closeGate());
			}
		}

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

		beamBreaker.update(intakeRunningIn);
		ballCount = beamBreaker.getBallCount();

		if (ballCount >= 3 && !prespinTriggered) {
			prespinTriggered = true;
		} else if (ballCount < 3 && prespinTriggered) {
			prespinTriggered = false;
		}

		displayTelemetry();

		if (gamepad1.xWasPressed()) {
			gamepad1.rumble(300);
			follower.setPose(PoseDatabase.getResetPose(getTeam()));
		}
	}

	@Override
	public void stop() {
	}

	protected abstract Pose getStartingPose();

	protected abstract Team getTeam();

	private void handleDriveInput() {
		if (follower.getPose().getY() >= 72.0) {
			upperShooterSpeed = Shooter.GOAL_RPM_UPPER;
			lowerShooterSpeed = Shooter.GOAL_RPM_LOWER;
		} else {
			upperShooterSpeed = Shooter.AUDIENCE_RPM_UPPER;
			lowerShooterSpeed = Shooter.AUDIENCE_RPM_LOWER;
		}

		if (gamepad1.a && !aButtonPressed) {
			brokeFollowing = false;
			follower.followPath(pathAudienceShoot.get(), true);
			aButtonPressed = true;
		} else if (!gamepad1.a && aButtonPressed) {
			aButtonPressed = false;
		} else if (gamepad1.b && !gamepad1BWasPressed) {
			brokeFollowing = false;
			follower.followPath(pathGoalShoot.get(), true);
			gamepad1BWasPressed = true;
		} else if (!gamepad1.b && gamepad1BWasPressed) {
			gamepad1BWasPressed = false;
		}

		if (!gamepad1.b && !gamepad1.a) {
			if (!brokeFollowing) {
				follower.breakFollowing();
				brokeFollowing = true;
			}

			if (getTeam() == Team.RED) {
				humanPlayerHeading = Math.toRadians(0);
				goalX = 129.717;
			} else if (getTeam() == Team.BLUE) {
				humanPlayerHeading = Math.toRadians(180);
				goalX = 15.024;
			}

			if (gamepad1.left_trigger > 0) {
				currentHeading = follower.getHeading();
				targetHeading = humanPlayerHeading;
				headingPIDController.setCoefficients(Constants.followerConstants.coefficientsHeadingPIDF);
				headingPIDController.updatePosition(currentHeading);
				headingDeadzone = 3;
			} else {
				if (limelight.goalsFound(getTeam())) {
					currentHeading = limelight.AngleFrom(getTeam());
					targetHeading = 0;
					headingPIDController.setCoefficients(new PIDFCoefficients(P, I, D, F));
					headingPIDController.updatePosition(currentHeading);
					headingDeadzone = 1;
				} else {
					currentHeading = follower.getHeading();
					targetHeading = limelight.calculateShotAngle(follower.getPose().getX(), follower.getPose().getY(), goalX, 130.927);
					headingPIDController.setCoefficients(Constants.followerConstants.coefficientsHeadingPIDF);
					headingPIDController.updatePosition(currentHeading);
					headingDeadzone = 3;
				}
			}

			headingLock = gamepad1.right_trigger > 0;

			if (headingLock || gamepad1.left_trigger > 0) {
				double headingError = targetHeading - currentHeading;
				headingError = Math.IEEEremainder(headingError, 2 * Math.PI);

				if (Math.abs(headingError) < Math.toRadians(headingDeadzone)) {
					headingCorrection = 0;
					if (headingLockRumbleSent) {
						gamepad2.rumble(300);
						headingLockRumbleSent = false;
					}
				} else {
					headingPIDController.setTargetPosition(targetHeading);
					headingCorrection = -headingPIDController.run();
				}

				writeDriveIfChanged(gamepad1.left_stick_x, gamepad1.left_stick_y, headingCorrection);
			} else {
				headingLockRumbleSent = false;
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

	protected void handleOperatorInput() {
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

		if (gamepad2.right_trigger > 0.5 && !rightTriggerPressed) {
			scheduler.schedule(
					new SequentialCommandGroup(
							shooter.SetTarget(upperShooterSpeed, lowerShooterSpeed),
							new WaitUntilCommand(() -> shooter.getPercentToTarget() >= 0.8),
							new InstantCommand(() -> openGate = true),
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
			openGate = false;
			rightTriggerPressed = false;
		}

		if (gamepad2.a && !gamepad2AWasPressed) {
			beamBreaker.resetBallCount();
			ballCount = 0;
			prespinTriggered = false;
			wasBeamAtThree = false;
			gamepad2AWasPressed = true;
		} else if (!gamepad2.a && gamepad2AWasPressed) {
			gamepad2AWasPressed = false;
		}

		if (gamepad2.yWasPressed() || (ballCount >= 3 && !prespinTriggered)) {
			scheduler.schedule(shooter.SetTarget(upperShooterSpeed, lowerShooterSpeed));
			prespinTriggered = true;
		}

		if (gamepad2.x && !xButtonPressed) {
			scheduler.schedule(transfer.TransferOut());
			xButtonPressed = true;
		} else if (!gamepad2.x && xButtonPressed && !rightTriggerPressed) {
			scheduler.schedule(transfer.TransferStop());
			xButtonPressed = false;
		}

		if (gamepad2.b && !gamepad2BWasPressed) {
			scheduler.schedule(transfer.IntakeDoorIn());
			scheduler.schedule(intake.Out());
			scheduler.schedule(conveyor.Out());
			gamepad2BWasPressed = true;
		} else if (!gamepad2.b && gamepad2BWasPressed) {
			scheduler.schedule(transfer.IntakeDoorStop());
			scheduler.schedule(intake.Stop());
			scheduler.schedule(conveyor.Stop());
			gamepad2BWasPressed = false;
		}

		double leftJoystickY = -gamepad2.left_stick_y;
		boolean inDeadzone = Math.abs(gamepad2.left_stick_y) < 0.2;

		if (inDeadzone) {
			if (!leftJoystickDeadzone) {
				scheduler.schedule(conveyor.Stop());
				scheduler.schedule(transfer.IntakeDoorStop());
				leftJoystickDeadzone = true;
			}
		} else {
			leftJoystickDeadzone = false;
			scheduler.schedule(new PerpetualCommand(conveyor.DirectPower(leftJoystickY)));

			if (leftJoystickY > 0) {
				scheduler.schedule(transfer.IntakeDoorOut());
			} else {
				scheduler.schedule(transfer.IntakeDoorIn());
			}
		}
	}

	protected void displayTelemetry() {
		if (RobotConfig.COMPETITION) return;

		panelsTelemetry.addData("Location", follower.getPose().toString());
		Drawing.drawRobot(follower.getPose());
		Drawing.sendPacket();

		panelsTelemetry.addData("Shooter At Target", transfer.reachedAverageTarget);

		panelsTelemetry.addData("Shooter Lower RPM", shooter.lowerRPM);
		panelsTelemetry.addData("Shooter Upper RPM", shooter.upperRPM);
		panelsTelemetry.addData("Shooter Lower Target", shooter.lowerTarget);
		panelsTelemetry.addData("Shooter Upper Target", shooter.upperTarget);

		beamBreaker.telemetry(panelsTelemetry);

		panelsTelemetry.update();

	}

	private void handleRumbleFeedback() {
		if (timer.seconds() >= 99 && !warnedEndGame) {
			gamepad1.rumbleBlips(3);
			gamepad2.rumbleBlips(3);
			warnedEndGame = true;
		}

		boolean isPathBusy = follower.isBusy();
		if (gamepad1.a && wasPathBusy && !isPathBusy) {
			gamepad2.rumble(100);
		}
		wasPathBusy = isPathBusy;

		if (transfer.reachedAverageTarget && !wasShooterAtTarget) {
			gamepad2.rumble(500);
		}
		wasShooterAtTarget = transfer.reachedAverageTarget;

		if (ballCount >= 3 && !wasBeamAtThree) {
			gamepad2.rumble(250);
		}
		wasBeamAtThree = ballCount >= 3;
	}
}
