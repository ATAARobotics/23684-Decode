package org.firstinspires.ftc.teamcode.OpModes.Auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.util.List;

import org.firstinspires.ftc.teamcode.LifecycleManagementUtilities.HardwareInitializer;
import org.firstinspires.ftc.teamcode.LifecycleManagementUtilities.SubsystemUpdater;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.RobotState;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.Utilities.ActionScheduler;
import org.firstinspires.ftc.teamcode.Utilities.SpindexerPosition;
import org.firstinspires.ftc.teamcode.Utilities.Transfer;

/**
 * Abstract base class for Audience-side autonomous modes.
 * Subclasses should implement alliance-specific constants and pose methods.
 */
public abstract class AudienceAuto extends OpMode {
	// Alliance-specific constants (implemented by subclasses)
	protected abstract double getShootingX();
	protected abstract double getShootingY();
	protected abstract double getGoalX();
	protected abstract double getGoalY();
	protected abstract Pose2d getStartPose();
	protected abstract Pose2d getInitialTrajectoryStartPose();
	protected abstract Vector2d getCollectionPosition();
	protected abstract double getCollectionHeading();
	protected abstract double GetShootingHeading();

	private Spindexer spindexer;
	private Shooter shooter;
	private org.firstinspires.ftc.teamcode.Subsystems.Transfer transfer;
	private Intake intake;
	private MecanumDrive drive;
	private TrajectoryActionBuilder trajectoryToShootingPosition;
	private TrajectoryActionBuilder trajectoryToCollectionPosition;

	private ShootThreeAction shootThreeAction;
	private int shooterTarget1;
	private int shooterTarget2;
	private int shooterTarget3;
	private ActionScheduler actionScheduler;
	private List<LynxModule> allHubs;

	protected double calculateShotAngle(double x, double y) {
		double deltaX = getGoalX() - x;
		double deltaY = getGoalY() - y;
		return Math.atan2(-deltaY, -deltaX);
	}

	@Override
	public void init() {
		telemetry.addData("Status", "Initializing subsystems...");
		telemetry.update();

		HardwareInitializer.initialize(hardwareMap);

		Shooter.initialize(hardwareMap);
		shooter = Shooter.getInstance();
		telemetry.addData("Subsystem Init", "Shooter initialized");
		telemetry.update();

		spindexer = Spindexer.getInstance();
		spindexer.resetCalibrationAverage();
		telemetry.addData("Subsystem Init", "Spindexer initialized");
		telemetry.update();

		transfer = org.firstinspires.ftc.teamcode.Subsystems.Transfer.getInstance();
		telemetry.addData("Subsystem Init", "Transfer initialized");
		telemetry.update();

		intake = Intake.getInstance();

		telemetry.addData("Subsystem Init", "Intake initialized");
		telemetry.update();

		Pose2d startPose = getStartPose();
		drive = new MecanumDrive(hardwareMap, startPose);
		telemetry.addData("Subsystem Init", "Drive initialized");
		telemetry.addData("Start Pose", "X: %.2f, Y: %.2f, Heading: %.2f°", startPose.position.x, startPose.position.y, Math.toDegrees(startPose.heading.toDouble()));
		telemetry.update();

		actionScheduler = ActionScheduler.getInstance();
		telemetry.addData("Subsystem Init", "ActionScheduler initialized");
		telemetry.update();

		// Enable Lynx bulk caching to reduce USB latency (~3ms per sensor call)
		allHubs = hardwareMap.getAll(LynxModule.class);
		for (LynxModule hub : allHubs) {
			hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
		}
		telemetry.addData("Subsystem Init", "Bulk Caching enabled on " + allHubs.size() + " hub(s)");
		telemetry.update();

		double shotAngle = GetShootingHeading();
		trajectoryToShootingPosition = drive.actionBuilder(new Pose2d(60, -9, Math.toRadians(0)))
				.strafeToLinearHeading(new Vector2d(55, -9), Math.toRadians(23));
		telemetry.addData("Trajectory", "To Shooting Position - Target: (%.1f, %.1f)", getShootingX(), getShootingY());
		telemetry.addData("Trajectory", "Shot angle: %.2f°", Math.toDegrees(shotAngle));
		telemetry.update();

		trajectoryToCollectionPosition = drive.actionBuilder(new Pose2d(55,-9, Math.toDegrees(23)))
				.strafeToLinearHeading(new Vector2d(35, -23), Math.toRadians(-90));
		telemetry.addData("Trajectory", "To Collection Position - Target: (%.1f, %.1f)", getCollectionPosition().x, getCollectionPosition().y);
		telemetry.addData("Trajectory", "Collection angle: %.2f°", getCollectionHeading());
		telemetry.update();

		shooterTarget1 = SpindexerPosition.getNextShootPosition(0);
		shooterTarget2 = SpindexerPosition.getNextShootPosition(shooterTarget1);
		shooterTarget3 = SpindexerPosition.getNextShootPosition(shooterTarget2);

		telemetry.addData("Status", "Initialization complete");
		telemetry.update();
	}

	@Override
	public void init_loop() {
		spindexer.updateCalibrationAverage();
	}

	@Override
	public void start() {
		telemetry.addData("Status", "Match started - scheduling autonomous sequence");
		telemetry.addData("Event", "Action Sequence", "1. Move to shooting position + spin up");
		telemetry.addData("Event", "Action Sequence", "2. Spindexer to intake position + fire");
		telemetry.addData("Event", "Action Sequence", "3. Maintain shooter RPM and fire");
		telemetry.addData("Event", "Action Sequence", "4. Repeat spin cycles");
		telemetry.addData("Event", "Action Sequence", "5. Move to collection position");
		telemetry.update();

		spindexer.finalizeTeleOpCalibration();

		actionScheduler.schedule(
				new SequentialAction(
						trajectoryToShootingPosition.build(),
						shootThreeAction.ShootThreeArtifacts(),
						trajectoryToCollectionPosition.build(),
						new SleepAction(3)
				)
		);

		telemetry.addData("Status", "Autonomous sequence scheduled and running");
		telemetry.update();
	}

	@Override
	public void loop() {
		SubsystemUpdater.update();
		actionScheduler.update();
		drive.updatePoseEstimate(); // Keep separate as drive-specific
		spindexer.update();

		// Control transfer mechanism based on readiness (spindexer position + shooter RPM)
		if (Transfer.isTransferReady(spindexer, shooter, Shooter.AUDIENCE_RPM)) {
			transfer.transferForward();
		} else {
			transfer.transferBackward();
		}

		// Update telemetry
		Pose2d currentPose = drive.localizer.getPose();
		telemetry.addData("Drive Position X", "%.2f", currentPose.position.x);
		telemetry.addData("Drive Position Y", "%.2f", currentPose.position.y);
		telemetry.addData("Drive Heading", "%.2f°", Math.toDegrees(currentPose.heading.toDouble()));
		telemetry.addData("Scheduler Status", !actionScheduler.isSchedulerEmpty() ? "Busy" : "Idle");
		telemetry.addData("Shooter RPM", "%.0f", shooter.averageRPM);
		telemetry.addData("Shooter Target RPM", "%.0f", Shooter.AUDIENCE_RPM);
		telemetry.addData("Spindexer at Shooting Pos", Transfer.isSpindexerAtShootingPosition(spindexer));
		telemetry.addData("Shooter at Target RPM", Transfer.isShooterAtTargetRPM(shooter, Shooter.AUDIENCE_RPM));
		telemetry.addData("Transfer Ready", Transfer.isTransferReady(spindexer, shooter, Shooter.AUDIENCE_RPM));
		telemetry.update();
	}

	@Override
	public void stop() {
		// Save the final pose for use in teleop
		Pose2d finalPose = drive.localizer.getPose();
		RobotState.getInstance().saveAutoPose(finalPose);
		telemetry.addData("Auto Stop", "Final pose saved: (%.2f, %.2f, %.2f°)", 
				finalPose.position.x, finalPose.position.y, Math.toDegrees(finalPose.heading.toDouble()));
		telemetry.update();
	}
}
