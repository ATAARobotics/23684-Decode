package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;
import org.firstinspires.ftc.teamcode.Subsystem.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystem.Transfer;
import org.firstinspires.ftc.teamcode.Utils.ShootArtifacts;
import org.firstinspires.ftc.teamcode.Utils.Team;

@Configurable
public abstract class AudienceAuto extends OpMode {
	// Configurable wait times (in milliseconds)
	public static int SHOOT_DWELL_TIME = 3200; // Time to allow all artifacts to be shot
	public static int SPIKE_COLLECTION_WAIT = 800; // Short wait during spike collection
	private final ElapsedTime timer = new ElapsedTime();
	public Follower follower;
	public Servo rgbIndicator;
	private CommandScheduler scheduler;
	private Intake intake;
	private Shooter shooter;
	private Spindexer spindexer;
	private Transfer transfer;
	private TelemetryManager panelsTelemetry;
	private Paths paths;

	double indicatorValue() {
		double x = timer.seconds();
		double hz;

		if (hardwareMap.voltageSensor.iterator().next().getVoltage() >= 13.5) {
			hz = 2.5;
		} else if (hardwareMap.voltageSensor.iterator().next().getVoltage() <= 13.5 && hardwareMap.voltageSensor.iterator().next().getVoltage() >= 12) {
			hz = 1;
		} else {
			hz = 0.5;
		}

		int state = Math.floorMod((int) Math.floor(x * hz), 2);
		return 0.23 * state + 0.388;
	}

	protected abstract Pose getStartingPose();

	protected abstract Team getTeam();

	@Override
	public void init() {
		panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

		follower = Constants.createFollower(hardwareMap);
		follower.setStartingPose(getStartingPose());

		scheduler = CommandScheduler.getInstance();
		scheduler.reset();
		scheduler.setBulkReading(hardwareMap, LynxModule.BulkCachingMode.AUTO);

		intake = new Intake(hardwareMap);
		shooter = new Shooter(hardwareMap);
		spindexer = new Spindexer(hardwareMap);
		transfer = new Transfer(hardwareMap);
		transfer.setShooter(shooter);
		transfer.setSpindexer(spindexer);

		paths = new Paths(follower, getTeam());

		rgbIndicator = hardwareMap.get(Servo.class, "rgbIndicator");

		panelsTelemetry.debug("Status", "Initialized");
		panelsTelemetry.update(telemetry);
	}

	@Override
	public void start() {
		spindexer.zeroSpindexer();
		timer.startTime();
		scheduler.schedule(
				new SequentialCommandGroup(
						new FollowPathCommand(follower, paths.shootPreload),

						new ShootArtifacts(shooter, spindexer, transfer, intake),
						// Turn off the motors and servos
						shooter.SetTarget(0, 0),
						intake.Stop(),
						transfer.TransferStop(),
						transfer.IntakeDoorStop(),

						new FollowPathCommand(follower, paths.toSpikeOne).setGlobalMaxPower(0.1),

						intake.In(),
						spindexer.DirectPower(0.4),
						transfer.IntakeDoorOut(),

						new WaitCommand(SPIKE_COLLECTION_WAIT), // Short wait during collection
						new FollowPathCommand(follower, paths.collectSpikeOne),
						new WaitCommand(SPIKE_COLLECTION_WAIT),

						new FollowPathCommand(follower, paths.toShootSpikeOne).setGlobalMaxPower(1),

						transfer.IntakeDoorStop(),
						intake.SlowOut(),
						spindexer.DirectPower(0),

						new ShootArtifacts(shooter, spindexer, transfer, intake),
						// Turn off the motors and servos
						shooter.SetTarget(0, 0),
						intake.Stop(),
						transfer.TransferStop(),
						transfer.IntakeDoorStop(),

						transfer.TransferIn(),
						new ParallelCommandGroup(
								spindexer.DirectPower(0.3),
								transfer.IntakeDoorOut(),
								intake.In()
						),
						new WaitCommand(SPIKE_COLLECTION_WAIT), // Short wait during collection
						new FollowPathCommand(follower, paths.collectSpikeTwo),
						new WaitCommand(SPIKE_COLLECTION_WAIT),
						transfer.TransferStop(),
						new ParallelCommandGroup(
								spindexer.DirectPower(0),
								transfer.IntakeDoorStop(),
								intake.SlowOut()
						),

						new FollowPathCommand(follower, paths.toParkSpikeTwo)

//						new FollowPathCommand(follower, paths.toShootSpikeTwo),
//
//						new ShootArtifacts(shooter, spindexer, transfer, intake),
//						// Turn off the motors and servos
//						shooter.SetTarget(0, 0),
//						intake.Stop(),
//						transfer.TransferStop(),
//						transfer.IntakeDoorStop(),
//
//						new FollowPathCommand(follower, paths.toSpikeThree),
//						transfer.TransferIn(),
//						new ParallelCommandGroup(
//							spindexer.DirectPower(0.3),
//							transfer.IntakeDoorOut(),
//							intake.In()
//						),
//						new WaitCommand(SPIKE_COLLECTION_WAIT), // Short wait during collection
//						new FollowPathCommand(follower, paths.toCollectSpikeThree),
//						new WaitCommand(SPIKE_COLLECTION_WAIT),
//						transfer.TransferStop(),
//						new ParallelCommandGroup(
//							spindexer.DirectPower(0),
//							transfer.IntakeDoorStop(),
//							intake.SlowOut()
//						),
//						new FollowPathCommand(follower, paths.toShootSpikeThree),
//
//						new ShootArtifacts(shooter, spindexer, transfer, intake),
//						// Turn off the motors and servos
//						shooter.SetTarget(0, 0),
//						intake.Stop(),
//						transfer.TransferStop(),
//						transfer.IntakeDoorStop()
				)
		);

		scheduler.run();
	}

	@Override
	public void loop() {
		follower.update();
		scheduler.run();
		shooter.periodic(); // TODO: Find out why it doesn't work without this
		transfer.periodic(); // TODO: Find out why it doesn't work without this
		rgbIndicator.setPosition(indicatorValue());

		panelsTelemetry.addLine("=== SHOOTER ===");
		panelsTelemetry.addData("Upper RPM", shooter.upperRPM);
		panelsTelemetry.addData("Lower RPM", shooter.lowerRPM);
		panelsTelemetry.addData("Average RPM", shooter.averageRPM);

		panelsTelemetry.addLine("=== TRANSFER ===");
		panelsTelemetry.addData("Shooter Lower At Target (This may be inactive, you may need to refer to \"At Target\")", transfer.reachedLowerTarget);
		panelsTelemetry.addData("Shooter Upper At Target (This may be inactive, you may need to refer to \"At Target\")", transfer.reachedUpperTarget);
		panelsTelemetry.addData("Shooter At Target (This may be inactive, you may need to refer to \"Lower At Target\" and \"Upper At Target\")", transfer.reachedAverageTarget);
		panelsTelemetry.addData("Spindexer At Target", transfer.spindexerAtTarget);
		panelsTelemetry.addData("Automatic Transfer Running?", transfer.runAutomaticTransfer);

		panelsTelemetry.update();
		panelsTelemetry.update(telemetry);
	}

	public static class Paths {
		private final Team team;
		public PathChain shootPreload;
		public PathChain toSpikeOne;
		public PathChain collectSpikeOne;
		public PathChain toShootSpikeOne;
		public PathChain toSpikeTwo;
		public PathChain collectSpikeTwo;
		public PathChain toParkSpikeTwo;
		public PathChain toShootSpikeTwo;
		public PathChain toSpikeThree;
		public PathChain toCollectSpikeThree;
		public PathChain toShootSpikeThree;

		public Paths(Follower follower, Team team) {
			this.team = team;
			shootPreload = follower
					.pathBuilder()
					.addPath(
							new BezierLine(pose(63.450, 9), pose(59.440, 17.328))
					)
					.setLinearHeadingInterpolation(
							heading(270),
							heading(290)
					)
					.build();

			toSpikeOne = follower
					.pathBuilder()
					.addPath(
							new BezierLine(pose(59.440, 17.328), pose(41.000, 40.50))
					)
					.setLinearHeadingInterpolation(
							heading(294.935),
							heading(180)
					)
					.build();

			collectSpikeOne = follower
					.pathBuilder()
					.addPath(
							new BezierLine(pose(41.000, 40.50), pose(9.000, 40.50))
					)
					.setConstantHeadingInterpolation(heading(180))
					.build();

			toShootSpikeOne = follower
					.pathBuilder()
					.addPath(
							new BezierLine(pose(9.000, 40.50), pose(59.440, 17.328))
					)
					.setLinearHeadingInterpolation(
							heading(180),
							heading(290)
					)
					.build();

			toSpikeTwo = follower
					.pathBuilder()
					.addPath(
							new BezierLine(pose(59.44, 17.328), pose(41.000, 65.500))
					)
					.setLinearHeadingInterpolation(
							heading(294.935),
							heading(180)
					)
					.build();

			collectSpikeTwo = follower
					.pathBuilder()
					.addPath(
							new BezierLine(pose(41.000, 65.500), pose(9.000, 65.500))
					)
					.setConstantHeadingInterpolation(heading(180))
					.build();

			toParkSpikeTwo = follower
					.pathBuilder()
					.addPath(
							new BezierLine(pose(9.000, 65.500), pose(38.2, 33.6))
					)
					.build();

			toShootSpikeTwo = follower
					.pathBuilder()
					.addPath(
							new BezierLine(pose(9.000, 65.500), pose(59.440, 17.328))
					)
					.setLinearHeadingInterpolation(
							heading(180),
							heading(290)
					)
					.build();
			toSpikeThree = follower
					.pathBuilder()
					.addPath(
							new BezierLine(pose(59.44, 17.328), pose(41.000, 89.500))
					)
					.setLinearHeadingInterpolation(
							heading(294.935),
							heading(180)
					)
					.build();

			toCollectSpikeThree = follower
					.pathBuilder()
					.addPath(
							new BezierLine(pose(41.000, 89.500), pose(15.000, 89.500))
					)
					.setConstantHeadingInterpolation(heading(180))
					.build();

			toShootSpikeThree = follower
					.pathBuilder()
					.addPath(
							new BezierLine(pose(15.000, 89.500), pose(59.000, 17.328))
					)
					.setLinearHeadingInterpolation(
							heading(180),
							heading(290)
					)
					.build();
		}

		private Pose pose(double x, double y) {
			if (team == Team.RED) {
				return new Pose(144 - x, y);
			}
			return new Pose(x, y);
		}

		private double heading(double degrees) {
			double radians = Math.toRadians(degrees);
			if (team == Team.RED) {
				return Math.PI - radians;
			}
			return radians;
		}
	}
}