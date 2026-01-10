package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;
import org.firstinspires.ftc.teamcode.Subsystem.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystem.Transfer;
import org.firstinspires.ftc.teamcode.PedroPathing.Constants;

@Autonomous
@Configurable
public class AudienceAuto extends OpMode {

	public Follower follower; // Pedro Pathing follower instance
	CommandScheduler scheduler;
	Intake intake;
	Shooter shooter;
	Spindexer spindexer;
	Transfer transfer;
	private TelemetryManager panelsTelemetry; // Panels Telemetry instance
	private Paths paths; // Paths defined in the Paths class

	@Override
	public void init() {
		panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

		follower = Constants.createFollower(hardwareMap);
		follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

		scheduler = CommandScheduler.getInstance();

		intake = new Intake(hardwareMap);
		shooter = new Shooter(hardwareMap);
		spindexer = new Spindexer(hardwareMap);
		transfer = new Transfer(hardwareMap);
		paths = new Paths(follower);

		panelsTelemetry.debug("Status", "Initialized");
		panelsTelemetry.update(telemetry);
	}

	@Override
	public void start() {
		scheduler.schedule(
				new SequentialCommandGroup(
						new ParallelCommandGroup(
								new FollowPathCommand(follower, paths.shootPreload),
								shooter.SetTarget(Shooter.AUDIENCE_RPM, Shooter.AUDIENCE_RPM),
								spindexer.DirectPower(1),
								transfer.IntakeDoorIn(),
								transfer.TransferIn(),
								intake.Slow()
						),
						transfer.TransferOut(),
						new ParallelCommandGroup(
								shooter.SetTarget(0, 0),
								intake.In(),
								transfer.TransferIn(),
								new FollowPathCommand(follower, paths.ToSpikeOne)
						),
						new WaitCommand(30),
						new FollowPathCommand(follower, paths.CollectSpikeOne),
						new ParallelCommandGroup(
								new FollowPathCommand(follower, paths.toShootSpikeOne),
								intake.Out(),
								shooter.WaitForTarget()
						),
						transfer.TransferOut(),
						new ParallelCommandGroup(
								shooter.WaitForTarget(),
								intake.In(),
								transfer.TransferIn(),
								new FollowPathCommand(follower, paths.toSpikeTwo)
						),
						new WaitCommand(30),
						new FollowPathCommand(follower, paths.CollectSpikeTwo),
						new ParallelCommandGroup(
								new FollowPathCommand(follower, paths.toShootSpikeTwo),
								intake.Out(),
								shooter.WaitForTarget()
						),
						transfer.TransferOut(),
						new ParallelCommandGroup(
								shooter.SetTarget(0, 0),
								intake.In(),
								transfer.TransferIn(),
								new FollowPathCommand(follower, paths.toSpikeThree)
						),
						new WaitCommand(30),
						new FollowPathCommand(follower, paths.toCollectSpikeTree),
						new ParallelCommandGroup(
								new FollowPathCommand(follower, paths.toShootSpikeThree),
								intake.Out(),
								shooter.WaitForTarget()
						),
						transfer.TransferOut()
				)
		);

	}

	@Override
	public void loop() {
		follower.update(); // Update Pedro Pathing
		scheduler.run();

		// Log values to Panels and Driver Station
		panelsTelemetry.debug("X", follower.getPose().getX());
		panelsTelemetry.debug("Y", follower.getPose().getY());
		panelsTelemetry.debug("Heading", follower.getPose().getHeading());
		panelsTelemetry.update(telemetry);
	}

	public static class Paths {
		public PathChain shootPreload;
		public PathChain ToSpikeOne;
		public PathChain CollectSpikeOne;
		public PathChain toShootSpikeOne;
		public PathChain toSpikeTwo;
		public PathChain CollectSpikeTwo;
		public PathChain toShootSpikeTwo;
		public PathChain toSpikeThree;
		public PathChain toCollectSpikeTree;
		public PathChain toShootSpikeThree;

		public Paths(Follower follower) {
			shootPreload = follower
					.pathBuilder()
					.addPath(
							new BezierLine(new Pose(63.000, 9), new Pose(59.440, 17.328))
					)
					.setLinearHeadingInterpolation(
							Math.toRadians(270),
							Math.toRadians(294.935)
					)
					.build();

			ToSpikeOne = follower
					.pathBuilder()
					.addPath(
							new BezierLine(new Pose(59.440, 17.328), new Pose(41.000, 35.000))
					)
					.setLinearHeadingInterpolation(
							Math.toRadians(294.935),
							Math.toRadians(180)
					)
					.build();

			CollectSpikeOne = follower
					.pathBuilder()
					.addPath(
							new BezierLine(new Pose(41.000, 35.000), new Pose(9.000, 35.000))
					)
					.setConstantHeadingInterpolation(Math.toRadians(180))
					.build();

			toShootSpikeOne = follower
					.pathBuilder()
					.addPath(
							new BezierLine(new Pose(9.000, 35.000), new Pose(59.440, 17.328))
					)
					.setLinearHeadingInterpolation(
							Math.toRadians(180),
							Math.toRadians(294.935)
					)
					.build();

			toSpikeTwo = follower
					.pathBuilder()
					.addPath(
							new BezierLine(new Pose(59.44, 17.328), new Pose(41.000, 60.000))
					)
					.setLinearHeadingInterpolation(
							Math.toRadians(294.935),
							Math.toRadians(180)
					)
					.build();

			CollectSpikeTwo = follower
					.pathBuilder()
					.addPath(
							new BezierLine(new Pose(41.000, 60.000), new Pose(9.000, 60.000))
					)
					.setConstantHeadingInterpolation(Math.toRadians(180))
					.build();

			toShootSpikeTwo = follower
					.pathBuilder()
					.addPath(
							new BezierLine(new Pose(9.000, 60.000), new Pose(59.440, 17.328))
					)
					.setLinearHeadingInterpolation(
							Math.toRadians(180),
							Math.toRadians(294.935)
					)
					.build();
			toSpikeThree = follower
					.pathBuilder()
					.addPath(
							new BezierLine(new Pose(59.44, 17.328), new Pose(41.000, 84.000))
					)
					.setLinearHeadingInterpolation(
							Math.toRadians(294.935),
							Math.toRadians(180)
					)
					.build();

			toCollectSpikeTree = follower
					.pathBuilder()
					.addPath(
							new BezierLine(new Pose(41.000, 84.000), new Pose(15.000, 84.000))
					)
					.setConstantHeadingInterpolation(Math.toRadians(180))
					.build();

			toShootSpikeThree = follower
					.pathBuilder()
					.addPath(
							new BezierLine(new Pose(15.000, 84.000), new Pose(59.000, 17.328))
					)
					.setLinearHeadingInterpolation(
							Math.toRadians(180),
							Math.toRadians(294.935)
					)
					.build();
		}
	}
}