package org.firstinspires.ftc.teamcode.OpModes.TeleOp;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.util.List;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.LifecycleManagementUtilities.HardwareInitializer;
import org.firstinspires.ftc.teamcode.LifecycleManagementUtilities.HardwareShutdown;
import org.firstinspires.ftc.teamcode.LifecycleManagementUtilities.SubsystemUpdater;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.RGBIndicator;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.Utilities.ActionScheduler;
import org.firstinspires.ftc.teamcode.Utilities.Angle;
import org.firstinspires.ftc.teamcode.Utilities.PIDFController;
import org.firstinspires.ftc.teamcode.Utilities.ShotAngle;
import org.firstinspires.ftc.teamcode.Utilities.Team;
import org.firstinspires.ftc.teamcode.Utilities.Transfer;

/**
 * Main TeleOp OpMode for driver control
 * <p>
 * Gamepad 1 (Driver):
 * - Left Stick: Drive forward/backward/strafe
 * - Right Stick: Turn left/right
 */
public class MainTeleOp extends OpMode {
	protected MecanumDrive drive;
	protected ActionScheduler scheduler;
	protected Shooter shooter;
	protected Intake intake;
	protected org.firstinspires.ftc.teamcode.Subsystems.Transfer transfer;
	protected Spindexer spindexer;
	protected Limelight limelight;
	protected RGBIndicator rgbIndicator;
	protected List<LynxModule> allHubs;
	DcMotorEx frontRight;
	DcMotorEx rearRight;
	DcMotorEx frontLeft;
	DcMotorEx rearLeft;

	protected PIDFController headingPIDF;

	// Button state tracking to prevent continuous input
	protected boolean leftTriggerPressed = false;
	protected boolean rightTriggerPressed = false;
	protected boolean xButtonPressed = false;
	protected boolean aButtonPressed = false;
	protected boolean bButtonPressed = false;
	protected boolean spindexerUpCrossed = false;
	protected boolean spindexerMidCrossed = false;
	protected boolean spindexerDownCrossed = false;
	protected boolean transferAboveRPM = false;

	// Performance monitoring
	private long maxLoopTime = 0;

	// Telemetry throttling
	public static boolean ENABLE_TELEMETRY = false;
	private long lastTelemetryTime = 0;
	private static final long TELEMETRY_UPDATE_INTERVAL = 100_000_000; // 100ms (roughly 100 loop ticks at 10ms/loop)
	private static final long MAX_LOOP_TIME_FOR_TELEMETRY = 25_000_000; // 25ms
	private long previousLoopTime = 0;

	private final double Hp = 0.1;
	private final double Hi = 0;
	private final double Hd = 0;
	private final double Hf = 0;
	private double prevHeading;

	@Override
	public void init() {
		// Initialize hardware
		HardwareInitializer.initialize(hardwareMap);
		drive = new MecanumDrive(hardwareMap, getStartingPose());
		scheduler = ActionScheduler.getInstance();
		shooter = Shooter.getInstance();
		intake = Intake.getInstance();
		transfer = org.firstinspires.ftc.teamcode.Subsystems.Transfer.getInstance();
		spindexer = Spindexer.getInstance();
		rgbIndicator = RGBIndicator.getInstance();
		limelight = new Limelight(hardwareMap);
		headingPIDF = new PIDFController(Hp,Hi,Hd,Hf);
		prevHeading = Math.toDegrees(drive.localizer.getPose().heading.toDouble());

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
		scheduler.schedule(transfer.intakeDoorForward());
		scheduler.schedule(transfer.transferOut());
		scheduler.update();
		limelight.start();

		scheduler.schedule(spindexer.setTarget(0));
	}

	@Override
	public void init_loop(){
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
		drive.updatePoseEstimate();
		limelight.update();
		handleDriveInput();
		handleOperatorInput();
		//handleHeadinglockDriveInput();

		// Phase 1: Core subsystem updates
		// SubsystemUpdater and scheduler MUST run every loop, but telemetry is throttled
		boolean sendTelemetry = true;
		SubsystemUpdater.update(sendTelemetry);
		scheduler.update(sendTelemetry);

		// Phase 2: Useful updates (budget: 10ms)
		if ((System.nanoTime() - startTime) < 10_000_000) {
			updateRGBIndicator();
		}

		// Phase 3: Non-critical updates (budget: 15ms)
		if (ENABLE_TELEMETRY && (System.nanoTime() - startTime) < 15_000_000) {
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
		HardwareShutdown.shutdown();
	}

	/**
	 * Check if telemetry should be updated based on throttling and loop time constraints.
	 * 
	 * @param currentLoopStart the start time of the current loop iteration (nanoTime)
	 * @return true if telemetry should be sent to the dashboard, false otherwise
	 */
	private boolean shouldUpdateTelemetry(long currentLoopStart) {
		if (!ENABLE_TELEMETRY) {
			return false;
		}
		
		// Don't update telemetry if previous loop exceeded budget
		if (previousLoopTime > MAX_LOOP_TIME_FOR_TELEMETRY) {
			return false;
		}
		
		// Update telemetry only at the specified interval
		if ((currentLoopStart - lastTelemetryTime) >= TELEMETRY_UPDATE_INTERVAL) {
			lastTelemetryTime = currentLoopStart;
			return true;
		}
		
		return false;
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
	 * Update RGB indicator color
	 */
	private void updateRGBIndicator() {
		scheduler.schedule(rgbIndicator.setColorAction(Transfer.isShooterAtTargetRPM(shooter, Shooter.AUDIENCE_RPM) ? "#00AB66" : "#FF2C2C"));
	}

	/**
	 * Handle driving input from gamepad1
	 */
	private void handleDriveInput() {
		if (gamepad1.a && !aButtonPressed) {
			if (getTeam() == Team.RED) {
//				scheduler.schedule(
//						drive.actionBuilder(new Pose2d(limelight.botPose.getPosition().x, limelight.botPose.getPosition().y, limelight.botPose.getOrientation().getYaw()))
//								.strafeToLinearHeading(new Vector2d(56.75, 10.25),Math.toRadians(-23))
//								.build()
//				);
			} else if (getTeam() == Team.BLUE) {
//				scheduler.schedule(
//						drive.actionBuilder(new Pose2d(limelight.botPose.getPosition().x, limelight.botPose.getPosition().y, limelight.botPose.getOrientation().getYaw()))
//								.strafeToLinearHeading(new Vector2d(56.75, -10.25),Math.toRadians(23))
//								.build()
//				);
			}
			aButtonPressed = true;
		} else if (!gamepad1.a && aButtonPressed) {
			aButtonPressed = false;
		}

		if (!gamepad1.a) {
			double y = -gamepad1.left_stick_y; // Y stick value is reversed
			double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
			double rx = gamepad1.right_stick_x;

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

	private void handleHeadinglockDriveInput() {
		double DEADZONE_THRESHOLD = 0.05;

		int tag = 20;

		DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
		DcMotor rearRight = hardwareMap.get(DcMotor.class, "rearRight");
		DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
		DcMotor rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");

		headingPIDF.setPID(Hp,Hi,Hd,Hf);
		Pose2d noApriltagTarget  = new Pose2d(0,0,0);

		if (getTeam() == Team.RED){
			noApriltagTarget = new Pose2d(0,0, ShotAngle.calculateShotAngle(drive.localizer.getPose().position.x,drive.localizer.getPose().position.y,-72,72));
			tag = 24;
		}else if (getTeam() == Team.BLUE){
			noApriltagTarget = new Pose2d(0,0, ShotAngle.calculateShotAngle(drive.localizer.getPose().position.x,drive.localizer.getPose().position.y,-72,-72));
			tag = 20;
		}

		double prevUnwrappedHeading = 0;
		prevUnwrappedHeading = Angle.unwrap(prevHeading, Math.toDegrees(drive.localizer.getPose().heading.toDouble()), prevUnwrappedHeading, 180);

		// Calculate power commands
		double strafePower  = gamepad1.left_stick_x;
		double forwardPower = -gamepad1.left_stick_y;
		double turnPower = 0;

		if (gamepad1.right_trigger == 1) {
			if (limelight.AreGoalsFound()){
				turnPower = headingPIDF.getOutput(
						limelight.TagXOffset(tag),
						0);
			}else{
				turnPower = headingPIDF.getOutput(
						-prevUnwrappedHeading,
						Math.toDegrees(noApriltagTarget.heading.toDouble())
				);
			}
		} else{
			turnPower = gamepad1.left_stick_x;
		}


		prevHeading = Math.toDegrees(drive.localizer.getPose().heading.toDouble());

		// Apply deadzone
		forwardPower = Math.abs(forwardPower) > DEADZONE_THRESHOLD ? forwardPower : 0;
		strafePower = Math.abs(strafePower) > DEADZONE_THRESHOLD ? strafePower : 0;
		turnPower = Math.abs(turnPower) > DEADZONE_THRESHOLD ? turnPower : 0;

//		PoseVelocity2d velocity = new PoseVelocity2d(
//				new Vector2d(forwardPower, strafePower),
//				turnPower
//		);
//		drive.setDrivePowers(velocity);


		double rotX = strafePower * Math.cos(-drive.localizer.getPose().heading.toDouble()) - forwardPower * Math.sin(-drive.localizer.getPose().heading.toDouble());
		double rotY = strafePower * Math.sin(-drive.localizer.getPose().heading.toDouble()) + forwardPower * Math.cos(-drive.localizer.getPose().heading.toDouble());

		rotX = rotX * 1.1;  // Counteract imperfect strafing

		// Denominator is the largest motor power (absolute value) or 1
		// This ensures all the powers maintain the same ratio,
		// but only if at least one is out of the range [-1, 1]
		double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turnPower), 1);
		double frontLeftPower = (rotY + rotX + turnPower) / denominator;
		double backLeftPower = (rotY - rotX + turnPower) / denominator;
		double frontRightPower = (rotY - rotX - turnPower) / denominator;
		double backRightPower = (rotY + rotX - turnPower) / denominator;

		frontLeft.setPower(frontLeftPower);
		rearLeft.setPower(backLeftPower);
		frontRight.setPower(frontRightPower);
		rearRight.setPower(backRightPower);
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
			scheduler.schedule(transfer.transferIn());
			xButtonPressed = true;
		} else if (!gamepad2.x && xButtonPressed) {
			scheduler.schedule(transfer.transferOut());
			xButtonPressed = false;
		}

		// Automatic transfer based on readiness (only if X button isn't held)
		if (!gamepad2.x) {
			boolean isReady = Transfer.isTransferReady(spindexer, shooter, Shooter.AUDIENCE_RPM);
			if (isReady && !transferAboveRPM) {
				scheduler.schedule(transfer.transferIn());
			} else if (!isReady && transferAboveRPM) {
				scheduler.schedule(transfer.transferOut());
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

		double spindexerPower = 0.4;

		if(gamepad2.right_bumper){
			spindexerPower = 0.8;
		}else{
			spindexerPower = 0.4;
		}

		// Crosses 0.2 threshold going up (from lower to 0.2+)
		 if (leftJoystickY >= 0.2 && !spindexerUpCrossed) {
			scheduler.schedule(spindexer.setDirectPower(spindexerPower));
			spindexerUpCrossed = true;
			spindexerMidCrossed = false;
			spindexerDownCrossed = false;
		}
		// Crosses -0.2 threshold going down (to -0.2 or below)
		else if (leftJoystickY <= -0.2 && !spindexerDownCrossed) {
			scheduler.schedule(spindexer.setDirectPower(-spindexerPower));
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

		telemetry.addData("Location", drive.localizer.getPose().toString());

		telemetry.addLine("=== GAMEPAD 2 (Operator) ===");
		telemetry.addData("Left Trigger", "Intake");
		telemetry.addData("Right Trigger", "Shooter");
		telemetry.addData("Left Joystick Y (Spindexer)", String.format("%.2f", -gamepad2.left_stick_y));

		telemetry.addLine("=== SHOOTER ===");
		telemetry.addData("Upper RPM", String.format("%.2f", shooter.upperRPM));
		telemetry.addData("Lower RPM", String.format("%.2f", shooter.lowerRPM));
		telemetry.addData("Average RPM", String.format("%.2f", shooter.averageRPM));

		telemetry.addLine("=== Transfer ===");
		telemetry.addData("Spindexer at Shooting Pos", Transfer.isSpindexerAtShootingPosition(spindexer));
		telemetry.addData("Shooter at Target RPM", Transfer.isShooterAtTargetRPM(shooter, Shooter.AUDIENCE_RPM));
		telemetry.addData("Transfer Ready", Transfer.isTransferReady(spindexer, shooter, Shooter.AUDIENCE_RPM));
	}
}
