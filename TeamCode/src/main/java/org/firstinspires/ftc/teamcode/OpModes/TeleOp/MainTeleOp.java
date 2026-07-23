package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.PerpetualCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.controller.SquIDFController;

import org.firstinspires.ftc.teamcode.OpModes.Auto.Modular.PoseDatabase;
import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.BeamBreaker;
import org.firstinspires.ftc.teamcode.Subsystem.Conveyor;
import org.firstinspires.ftc.teamcode.Subsystem.Gate;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
//import org.firstinspires.ftc.teamcode.Subsystem.Limelight;
import org.firstinspires.ftc.teamcode.Subsystem.RGBIndicator;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;
import org.firstinspires.ftc.teamcode.Subsystem.Transfer;
import org.firstinspires.ftc.teamcode.Utils.Drawing;
import org.firstinspires.ftc.teamcode.Utils.RobotConfig;
import org.firstinspires.ftc.teamcode.Utils.RobotPosition;
import org.firstinspires.ftc.teamcode.Utils.ShootingZone;
import org.firstinspires.ftc.teamcode.Utils.Team;
import org.firstinspires.ftc.teamcode.Utils.Drive;

import java.util.function.Supplier;

@Configurable
public abstract class MainTeleOp extends OpMode {
	protected Follower follower;
	protected CommandScheduler scheduler;
	protected Shooter shooter;
	protected Intake intake;
	protected Transfer transfer;
	protected Conveyor conveyor;
//	protected Limelight limelight;
	protected Gate gate;
	protected BeamBreaker beamBreaker;
	protected RGBIndicator rgbIndicator;
	protected int ballCount = 0;
	protected boolean lastIntakeIn = true;
	protected boolean prespinTriggered = false;
	protected boolean prespinTriggeredAtTwo = false;
	protected boolean prespinTriggeredByAlign = false;
	protected TelemetryManager.TelemetryWrapper panelsTelemetry;
	protected boolean leftTriggerPressed = false;
	protected boolean rightTriggerPressed = false;
	protected boolean aButtonPressed = false;
	protected boolean gamepad1BWasPressed = false;
	protected boolean gamepad2BWasPressed = false;
	protected boolean leftJoystickDeadzone = true;
	protected boolean brokeFollowing = false;
	protected boolean wasShooterAtTarget = false;
	protected boolean wasPathBusy = false;
	protected boolean warnedEndGame = false;
	protected boolean headingLockRumbleSent = false;
	protected boolean wasBeamAtTwo = false;
	/**
	 * True after the operator pressed gamepad2 Y ("stop prespin"). While true,
	 * the ball-count and align-triggered auto-prespin paths are suppressed.
	 * Cleared by the next shot (gamepad2 right trigger pressed) or the next
	 * manual prespin (gamepad2 left bumper pressed).
	 */
	protected boolean prespinStoppedByUser = false;
	/**
	 * True while gamepad1 Y is held. While held, drivetrain commands are
	 * zeroed so the robot holds its current pose.
	 */
	protected boolean positionHold = false;
	/**
	 * Edge-tracking for gamepad1 right bumper (drive-to-nearest-zone). The
	 * path runs at full power while the bumper is held; releasing it
	 * cancels the path. When the path completes naturally the shooter
	 * prespins automatically.
	 */
	protected boolean driveToZoneActive = false;
	/**
	 * True when the drive-to-zone path finished (or the robot was already
	 * inside a zone when the bumper was pressed). Cleared once the bumper
	 * is released. Used so the path doesn't re-trigger the moment the
	 * follower reports idle.
	 */
	protected boolean driveToZoneArrived = false;
	/**
	 * True after one of our TeleOp-triggered paths (audience shoot A, goal
	 * shoot B, drive-to-zone bumper) was started. Used to detect the
	 * "path completed naturally" case where {@code follower.isBusy()}
	 * returns false but the follower is still in holdEnd position-hold and
	 * will fight manual stick input until {@code breakFollowing()} runs.
	 */
	protected boolean autoPathActive = false;
	/**
	 * True while gamepad2 right bumper is held and the robot satisfies the
	 * shoot-when-positioned gates. Latches once so we don't repeatedly
	 * schedule the shoot sequence while the bumper stays held.
	 */
	protected boolean shootWhenHeldActive = false;
	/**
	 * True while the shoot-when-held feeder (transfer + conveyor) is running.
	 * Tracks whether the gate is currently feeding so that when the gates
	 * momentarily fail and then re-pass, we can restart the feeder without
	 * waiting on the full SequentialCommandGroup to re-fire.
	 */
	protected boolean shootFeederRunning = false;
	/**
	 * Aim tolerance for the gamepad2-right-bumper shoot gate. Tuned as 5°
	 * (loose enough to feel snappy, tight enough to keep balls landing in
	 * the goal). Read fresh each frame so dashboard edits take effect.
	 */
	public static double SHOOT_AIM_TOLERANCE_DEG = 5.0;
	/**
	 * Signed-distance margin (inches) the robot center must be INSIDE a
	 * shooting zone before the shoot-when-held gate fires. The "≥ 1 in
	 * inside" requirement from the driver.
	 */
	public static double SHOOT_ZONE_INSIDE_MARGIN_IN = 1.0;
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
	private Supplier<PathChain> pathDriveToZone;
	double upperShooterSpeed = Shooter.AUDIENCE_RPM_UPPER;
	double lowerShooterSpeed = Shooter.AUDIENCE_RPM_LOWER;
    PIDFController headingPIDController;
	SquIDFController limelightPIDController;
	Drive drive;
	boolean headingLock = true;
	double currentHeading;
	double headingDeadzone;
	double targetHeading = Math.toRadians(180);

	double tar;
	double headingCorrection;

	double correctionspeed;
	double humanPlayerHeading = 0;

	public double anglewrap(double degrees){
		while(degrees >180 ) degrees -= 360;
		while (degrees < -180) degrees +=360;
		return degrees;
	}


	long headinglocktime = 0;
    //Timing.Timer Headinglocktimer = new Timing.Timer(headinglocktime);

	Timer headinglocktimer;
	double goalX = 0;
	boolean openGate = false;
	public static double P = 0.06, I, D = 0.002, F = 0.0001;

	private  double P2 = 0.03, I2, D2 = 0.0003, F2 = 0.001;

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
		drive = new Drive(hardwareMap);

		intake = new Intake(hardwareMap);
		transfer = new Transfer(hardwareMap);
		conveyor = new Conveyor(hardwareMap);
		beamBreaker = new BeamBreaker(hardwareMap);
		gate = new Gate(hardwareMap);
//		limelight = new Limelight(hardwareMap, follower);
		transfer.setShooter(shooter);

		headinglocktimer = new Timer();

		rgbIndicator = new RGBIndicator(hardwareMap);
		rgbIndicator.setTransfer(transfer);
		rgbIndicator.setBeamBreaker(beamBreaker);
		rgbIndicator.setFollower(follower);
		rgbIndicator.setTeam(getTeam());

		// Repoint at the post-reset scheduler singleton so scheduler.run() drives
		// the subsystems registered above on the new instance.
		scheduler = CommandScheduler.getInstance();

		panelsTelemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();
		headingPIDController = new PIDFController(new PIDFCoefficients(P2, I2, D2, F2));

		limelightPIDController = new SquIDFController(P,I,D,F);

		if (!RobotConfig.COMPETITION) {
			telemetry.addData("Status", "Initialized - Waiting for START");
			telemetry.update();
		}

		pathAudienceShoot = () -> buildShootPath(new Pose(33.098, 43.424), Math.toRadians(180));
		pathGoalShoot = () -> buildShootPath(PoseDatabase.getGoalShootPose(getTeam()), Math.toRadians(340.7));
		pathDriveToZone = () -> buildDriveToZonePath();
	}

	/**
	 * True when the robot center is already inside either alliance shooting
	 * zone triangle. Used to short-circuit {@link #buildDriveToZonePath()}
	 * when there is nothing to drive to.
	 */
	private boolean isAlreadyInZone() {
		Team team = getTeam();
		Pose pose = follower.getPose();
		double signed = (team == Team.RED)
				? ShootingZone.signedDistanceIntoRedZone(pose)
				: ShootingZone.signedDistanceIntoBlueZone(pose);
		return signed >= 0.0;
	}

	/**
	 * Builds a BezierLine from the current pose to the closest point on either
	 * BLUE/RED shooting-zone triangle (whichever is nearer). Endpoint heading
	 * is set to the aim heading for the alliance-specific goal so the robot
	 * arrives already pointed at the goal — on arrival, the driver only has to
	 * release the bumper to stop and the shooter is already spinning.
	 * <p>
	 * Returns {@code null} when the robot center is already inside a zone,
	 * since {@code BezierLine(current, current)} is degenerate and Pedro's
	 * closest-point math divides by the segment length.
	 */
	private PathChain buildDriveToZonePath() {
		if (isAlreadyInZone()) {
			return null;
		}
		Team team = getTeam();
		double goalX = (team == Team.RED) ? 141.5 : 0.0;
		double goalY = 141.5;
		Pose current = follower.getPose();
		Pose entry = (team == Team.RED)
				? ShootingZone.nearestEntryInRedZone(current)
				: ShootingZone.nearestEntryInBlueZone(current);
		double endHeadingRad = ShootingZone.calculateAimHeading(
				entry.getX(), entry.getY(), goalX, goalY);
		return follower.pathBuilder()
				.addPath(new BezierLine(current, entry))
				.setLinearHeadingInterpolation(current.getHeading(), endHeadingRad, 0.8)
				.setNoDeceleration()
				.build();
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
		// If a TeleOp-triggered path was cancelled by the driver (A / B / right
		// bumper release) OR completed naturally with holdEnd, keep the follower
		// fully disengaged every loop. follower.update() would otherwise re-apply
		// holdEnd position-hold corrections on the next loop, which beat
		// writeDriveIfChanged's snap-threshold optimization and snap the robot
		// back to its last pose the moment the driver holds the joystick steady.
		boolean wantManual = brokeFollowing
				|| (autoPathActive && !follower.isBusy());
		if (wantManual && !driveToZoneActive) {
			follower.breakFollowing();
			if (autoPathActive && !follower.isBusy()) {
				autoPathActive = false;
				brokeFollowing = true;
			}
		}
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
		} else if (ballCount >= 2 && !prespinTriggeredAtTwo && !prespinTriggered) {
			prespinTriggered = true;
			prespinTriggeredAtTwo = true;
		} else if (ballCount < 2) {
			prespinTriggered = false;
			prespinTriggeredAtTwo = false;
		} else if (ballCount < 3 && prespinTriggeredAtTwo) {
			// Allow retrigger on 3 if we previously triggered on 2
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
			autoPathActive = true;
		} else if (!gamepad1.a && aButtonPressed) {
			aButtonPressed = false;
			follower.breakFollowing();
			autoPathActive = false;
			brokeFollowing = true;
		} else if (gamepad1.b && !gamepad1BWasPressed) {
			brokeFollowing = false;
			follower.followPath(pathGoalShoot.get(), true);
			gamepad1BWasPressed = true;
			autoPathActive = true;
		} else if (!gamepad1.b && gamepad1BWasPressed) {
			gamepad1BWasPressed = false;
			follower.breakFollowing();
			autoPathActive = false;
			brokeFollowing = true;
		}

		// gamepad1 right bumper (held) = drive at full power to the closest
		// shooting-zone entry, ending already aimed at the alliance goal. On
		// arrival the shooter prespins automatically. Releasing the bumper
		// cancels the path. The path itself sets maxPower=1 so the driver
		// doesn't need to also press the heading-lock triggers.
		if (gamepad1.right_bumper) {
			if (!driveToZoneActive) {
				// Edge: start the path. Re-evaluate entry each press so a
				// driver who backs up and re-presses gets a fresh path.
				driveToZoneActive = true;
				driveToZoneArrived = false;
				brokeFollowing = false;
				aButtonPressed = false;
				gamepad1BWasPressed = false;
				PathChain toZone = pathDriveToZone.get();
				if (toZone == null) {
					// Robot is already inside a zone — skip path scheduling
					// (BezierLine degenerate) and treat as immediately arrived.
					driveToZoneArrived = true;
					scheduler.schedule(shooter.SetTarget(upperShooterSpeed, lowerShooterSpeed));
					prespinStoppedByUser = false;
					writeDriveIfChanged(0, 0, 0);
				} else {
					follower.followPath(toZone, true);
					follower.setMaxPower(1.0);
					// Prespin immediately so the shooter is at speed by the time
					// we arrive at the zone. The stop-prespin latch (Y) can
					// still cancel this later.
					scheduler.schedule(shooter.SetTarget(upperShooterSpeed, lowerShooterSpeed));
					prespinStoppedByUser = false;
				}
			} else if (!follower.isBusy() && !driveToZoneArrived) {
				driveToZoneArrived = true;
				// Path finished (or we were already in the zone). Prespin
				// idempotently in case the SetTarget above lost the race.
				scheduler.schedule(shooter.SetTarget(upperShooterSpeed, lowerShooterSpeed));
				prespinStoppedByUser = false;
				writeDriveIfChanged(0, 0, 0);
			} else if (follower.isBusy()) {
				// Still driving to zone — don't overwrite the follower's
				// motor commands with stick input.
				return;
			} else {
				writeDriveIfChanged(0, 0, 0);
			}
		} else if (driveToZoneActive) {
			// Bumper was released. Cancel the path and reset the latches so
			// the next press starts fresh from the current pose. Always break,
			// not just when isBusy(): a completed holdEnd path leaves the
			// follower in position hold with isBusy()==false, and we still
			// need to stop its corrections.
			follower.breakFollowing();
			driveToZoneActive = false;
			driveToZoneArrived = false;
			brokeFollowing = true;
		}

		if (!gamepad1.b && !gamepad1.a && !driveToZoneActive) {

			if (!brokeFollowing && follower.isBusy()) {
				follower.breakFollowing();
				brokeFollowing = true;
			}

			// gamepad1 Y (held) = position hold. Zeroes the drivetrain and parks
			// the follower so the robot keeps its current pose. Y is only read
			// when neither audience-shoot (A) nor goal-shoot (B) is engaged.
			positionHold = gamepad1.y;
			if (positionHold) {
				if (follower.isBusy()) {
					follower.breakFollowing();
				}
				writeDriveIfChanged(0, 0, 0);
				return;
			}

			if (getTeam() == Team.RED) {
				humanPlayerHeading = MathFunctions.normalizeAngle(Math.toRadians(180));
				goalX = 141.5;
			} else if (getTeam() == Team.BLUE) {
				humanPlayerHeading = MathFunctions.normalizeAngle(Math.toRadians(0));
				goalX = 0;
			}

			if (gamepad1.left_trigger > 0) {
				currentHeading = follower.getHeading();
				tar = drive.calculateShotAngle(follower.getPose().getX(), follower.getPose().getY(), goalX, 141.5);
				targetHeading = 0;
				headingPIDController.setCoefficients(Drive.coefficientsHeadingPIDF);
				headingPIDController.updateError(anglewrap(Math.toDegrees(tar - currentHeading)));
				correctionspeed = -headingPIDController.run();
				headingDeadzone = 7;
			}

//		} else {
//				if (limelight.goalsFound(getTeam())) {
//					headinglocktimer.resetTimer();
//
//					currentHeading = limelight.AngleFrom(getTeam());
//					targetHeading = 0;
//					limelightPIDController.setPIDF(P, I, D, F);
//					headingDeadzone = 1;
//					correctionspeed = -limelightPIDController.calculate(currentHeading, targetHeading);
//
//				} else if (!limelight.goalsFound(getTeam()) && headinglocktimer.getElapsedTime() >= 300) {
//					currentHeading = follower.getHeading();
//					tar = drive.calculateShotAngle(follower.getPose().getX(), follower.getPose().getY(), goalX, 141.5);
//					targetHeading = 0;
//
//					if (Math.abs(Math.toDegrees(tar - currentHeading)) > 70) {
//						correctionspeed = -headingPIDController.run();
//					} else {
//						correctionspeed = MathFunctions.clamp(-headingPIDController.run(), -0.5, 0.5);
//					}
//					headingPIDController.setCoefficients(Drive.coefficientsHeadingPIDF);
//					headingPIDController.updateError(anglewrap(Math.toDegrees(tar - currentHeading)));
//					headingPIDController.updateFeedForwardInput(1);
//					headingDeadzone = 7;
//				}
//
//			}


				headingLock = gamepad1.right_trigger > 0;

				if (headingLock || gamepad1.left_trigger > 0) {
					double headingError = targetHeading - currentHeading;
					headingError = Math.IEEEremainder(headingError, 2 * Math.PI);

					double ori = 1;
					//if (Math.abs(headingError) > 180) ori = 1;
//				else ori = -1;
					if (Math.abs(headingError) < Math.toRadians(headingDeadzone)) {
						headingCorrection = 0;
						if (headingLockRumbleSent) {
							gamepad2.rumble(300);
							headingLockRumbleSent = false;
						}
					} else {
						headingCorrection = correctionspeed;
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
		drive.TeleopDrive(follower, x, y, h);
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
			// Operator pulled the shot trigger — clear the user-stopped-prespin
			// latch so the next auto-prespin cycle can run again.
			prespinStoppedByUser = false;
			if (ballCount > 0) {
				beamBreaker.resetBallCount();
				ballCount = 0;
				prespinTriggered = false;
				prespinTriggeredAtTwo = false;
				prespinTriggeredByAlign = false;
				wasBeamAtTwo = false;
			}
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
				prespinTriggeredAtTwo = false;
				prespinTriggeredByAlign = false;
				wasBeamAtTwo = false;
			}
			openGate = false;
			rightTriggerPressed = false;
		}

		boolean alignPressed = gamepad1.right_trigger > 0.5 || gamepad1.left_trigger > 0.5;

		// Manual prespin: gamepad2 left bumper (edge-triggered). Always allowed,
		// even when the operator previously hit Y to stop prespin — pressing the
		// bumper is the explicit "I want the shooter spinning again" intent and
		// also clears the stop latch.
		if (gamepad2.leftBumperWasPressed()) {
			scheduler.schedule(shooter.SetTarget(upperShooterSpeed, lowerShooterSpeed));
			prespinStoppedByUser = false;
		}

		// Stop prespin: gamepad2 Y (edge-triggered). Spins the shooter down and
		// latches prespinStoppedByUser so the ball-count and align auto-prespin
		// paths stay suppressed until the next shot or manual prespin.
		if (gamepad2.yWasPressed()) {
			scheduler.schedule(shooter.SetTarget(0, 0));
			prespinStoppedByUser = true;
		}

		// Auto prespin: ballCount threshold or align-trigger, but ONLY when the
		// operator hasn't latched a stop. Manual prespin (above) bypasses this
		// gate by virtue of running on left_bumper, not these conditions.
		if (!prespinStoppedByUser
				&& ((ballCount >= 2 && prespinTriggered)
					|| (alignPressed && ballCount > 0 && !prespinTriggeredByAlign))) {
			scheduler.schedule(shooter.SetTarget(upperShooterSpeed, lowerShooterSpeed));
			prespinTriggered = false;
			if (alignPressed) {
				prespinTriggeredByAlign = true;
			}
		} else if (!alignPressed && prespinTriggeredByAlign) {
			prespinTriggeredByAlign = false;
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

		// Shoot-when-held: gamepad2 right bumper, gated on the robot being
		// well-inside a shooting zone AND aimed at the alliance goal. While
		// held, run the same sequence as the right-trigger shot (prespin,
		// open gate at target, transfer + conveyor). On release, stop
		// everything and close the gate. The first time the gates pass we
		// schedule the sequence; while the bumper stays held we keep the
		// gate open and the feeder running via the openGate latch.
		if (gamepad2.right_bumper) {
			if (!shootWhenHeldActive && shootWhenHeldReady()) {
				shootWhenHeldActive = true;
				shootFeederRunning = false;
				prespinStoppedByUser = false;
				scheduler.schedule(
						new SequentialCommandGroup(
								shooter.SetTarget(upperShooterSpeed, lowerShooterSpeed),
								new WaitUntilCommand(() -> shooter.getPercentToTarget() >= 0.8
										&& shootWhenHeldReady()),
								new InstantCommand(() -> openGate = true),
								shooter.WaitForTarget().withTimeout(2500),
								transfer.TransferOut(),
								conveyor.In()
						));
				// Mark feeder as running on the next loop so we don't race
				// the SequentialCommandGroup's terminal TransferOut/In.
				shootFeederRunning = true;
			} else if (shootWhenHeldActive && shootWhenHeldReady()) {
				// Keep the gate open + transfer/conveyor on while held.
				if (!openGate) {
					openGate = true;
				}
				// If the gates were lost and the feeder was stopped while
				// the bumper stayed held, restart it now that the gates
				// pass again.
				if (!shootFeederRunning) {
					scheduler.schedule(transfer.TransferOut());
					scheduler.schedule(conveyor.In());
					shootFeederRunning = true;
				}
			} else if (shootWhenHeldActive && !shootWhenHeldReady()) {
				// Lost the gates while the bumper was still held — close
				// the gate and stop the feeder so we don't dribble.
				openGate = false;
				scheduler.schedule(transfer.TransferStop());
				scheduler.schedule(conveyor.Stop());
				shootFeederRunning = false;
			}
		} else if (shootWhenHeldActive) {
			// Bumper released — stop everything and reset the latch.
			shootWhenHeldActive = false;
			openGate = false;
			shootFeederRunning = false;
			scheduler.schedule(shooter.SetTarget(0, 0));
			scheduler.schedule(transfer.TransferStop());
			scheduler.schedule(conveyor.Stop());
		}
	}

	/**
	 * Combined gate check for the gamepad2 right-bumper shoot-when-held path.
	 * Requires the robot footprint to overlap an alliance shooting zone and the
	 * back-mounted shooter to be aimed within the configured goal tolerance.
	 */
	private boolean shootWhenHeldReady() {
		Team team = getTeam();
		Pose pose = follower.getPose();
		boolean inZone = (team == Team.RED)
				? ShootingZone.isAnyCornerInRedZone(pose)
				: ShootingZone.isAnyCornerInBlueZone(pose);
		double goalX = (team == Team.RED) ? 141.5 : 0.0;
		return inZone && ShootingZone.isAimedAtGoal(
				pose, goalX, 141.5, Math.toRadians(SHOOT_AIM_TOLERANCE_DEG));
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

		//limelight.Telemetry(telemetry);
		//telemetry.addData("ballcount", ballCount);
		//telemetry.addData("prespin Tiriggered?",prespinTriggered);
		telemetry.addLine(follower.getPose().toString());
		telemetry.addData("error",Math.toDegrees(tar - follower.getHeading()));
		telemetry.update();


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

		if (ballCount >= 2 && !wasBeamAtTwo) {
			gamepad2.rumble(250);
		}
		wasBeamAtTwo = ballCount >= 2;
	}
}
