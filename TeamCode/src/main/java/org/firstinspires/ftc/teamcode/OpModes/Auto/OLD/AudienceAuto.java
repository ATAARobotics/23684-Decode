package org.firstinspires.ftc.teamcode.OpModes.Auto.OLD;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;
import org.firstinspires.ftc.teamcode.Subsystem.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystem.Touch;
import org.firstinspires.ftc.teamcode.Subsystem.Transfer;
import org.firstinspires.ftc.teamcode.Utils.RobotPosition;
import org.firstinspires.ftc.teamcode.Utils.ShootArtifacts;
import org.firstinspires.ftc.teamcode.Utils.Team;

@Configurable
public abstract class AudienceAuto extends OpMode {
	private final ElapsedTime timer = new ElapsedTime();
	public Follower follower;
	public Paths paths;
	private CommandScheduler scheduler;
	private Intake intake;
	private Shooter shooter;
	private Spindexer spindexer;
	private Transfer transfer;

	Touch touch;
	private TelemetryManager panelsTelemetry;

	@Override
	public void stop() {
		if (follower != null) {
			RobotPosition.robotPose = follower.getPose();
			RobotPosition.isPoseSet = true;
		}
		if (spindexer != null) {
			spindexer.savePosition();
		}
		CommandScheduler.getInstance().reset();
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
		touch = new Touch(hardwareMap);
		touch.init();

		paths = new Paths(follower, getTeam());

		panelsTelemetry.debug("Status", "Initialized");
		panelsTelemetry.update(telemetry);
	}

	@Override
	public void start() {
		spindexer.zeroSpindexer();
		timer.startTime();
		scheduler.schedule(
				new SequentialCommandGroup(
						new ParallelCommandGroup(
								new FollowPathCommand(follower, paths.shootPreload),
								shooter.SetTarget(Shooter.AUDIENCE_RPM, Shooter.AUDIENCE_RPM)
						),
						transfer.SetAutomaticTransfer(true),
						new ShootArtifacts(shooter, spindexer, transfer, intake, touch),
						transfer.SetAutomaticTransfer(false),
						// Turn off the motors and servos
						shooter.SetTarget(0, 0),
						intake.Stop(),
						transfer.TransferStop(),
						transfer.IntakeDoorStop(),

//						new FollowPathCommand(follower, paths.toSpikeOne),
//						intake.In(),
//						spindexer.DirectPower(0.3),
//						transfer.IntakeDoorOut(),
//
//						new FollowPathCommand(follower, paths.collectSpikeOne),
//
//						intake.Slow(),
//						shooter.SetTarget(Shooter.AUDIENCE_RPM, Shooter.AUDIENCE_RPM),
//						spindexer.DirectPower(0),
//						new FollowPathCommand(follower, paths.toShootSpikeOne),
//						transfer.IntakeDoorOut(),
//
//						transfer.SetAutomaticTransfer(true),
//						new ShootArtifacts(shooter, spindexer, transfer, intake, touch),
//						transfer.SetAutomaticTransfer(false),
//						// Turn off the motors and servos
//						shooter.SetTarget(0, 0),
//						intake.Stop(),
//						transfer.TransferStop(),
//						transfer.IntakeDoorStop(),

						new FollowPathCommand(follower, paths.toSpikeTwo),

						transfer.TransferIn(),
						new ParallelCommandGroup(
								spindexer.DirectPower(0.3),
								transfer.IntakeDoorOut(),
								intake.In()
						),

						new FollowPathCommand(follower, paths.collectSpikeTwo),


						shooter.SetTarget(Shooter.AUDIENCE_RPM, Shooter.AUDIENCE_RPM),
						transfer.TransferStop(),
						new ParallelCommandGroup(
								spindexer.DirectPower(0),
								intake.Slow()
						),

						new FollowPathCommand(follower, paths.toShootSpikeTwo),
						transfer.IntakeDoorOut(),
						transfer.SetAutomaticTransfer(true),
						new ShootArtifacts(shooter, spindexer, transfer, intake, touch),
						transfer.SetAutomaticTransfer(false),
						// Turn off the motors and servos
						shooter.SetTarget(0, 0),
						intake.Stop(),
						transfer.TransferStop(),
						transfer.IntakeDoorStop(),

						new FollowPathCommand(follower, paths.toSpikeThree),

						transfer.TransferIn(),
						new ParallelCommandGroup(
								spindexer.DirectPower(0.3),
								transfer.IntakeDoorOut(),
								intake.In()
						),
						new FollowPathCommand(follower, paths.toCollectSpikeThree),

						shooter.SetTarget(0, 0),
						transfer.TransferStop(),
						new ParallelCommandGroup(
								spindexer.DirectPower(0),

								intake.SlowOut()
						),

						new FollowPathCommand(follower, paths.toShootSpikeThree),
						transfer.IntakeDoorOut(),
						transfer.SetAutomaticTransfer(true),
						new ShootArtifacts(shooter, spindexer, transfer, intake, touch),
						transfer.SetAutomaticTransfer(false),
						// Turn off the motors and servos
						shooter.SetTarget(0, 0),
						intake.Stop(),
						transfer.TransferStop(),
						transfer.IntakeDoorStop()
				)
		);

		scheduler.run();
	}

	@Override
	public void loop() {
		follower.update();
		scheduler.run();
		shooter.periodic();
		transfer.periodic();
		touch.Update(transfer);
		touch.Telemetry(telemetry);

		panelsTelemetry.addLine("=== SHOOTER ===");
		panelsTelemetry.addData("Upper RPM", shooter.upperRPM);
		panelsTelemetry.addData("Lower RPM", shooter.lowerRPM);
		panelsTelemetry.addData("Average RPM", shooter.averageRPM);

		panelsTelemetry.addLine("=== TRANSFER ===");
		panelsTelemetry.addData("Shooter Lower At Target", transfer.reachedLowerTarget);
		panelsTelemetry.addData("Shooter Upper At Target", transfer.reachedUpperTarget);
		panelsTelemetry.addData("Shooter At Target", transfer.reachedAverageTarget);
		panelsTelemetry.addData("Spindexer At Target", transfer.spindexerAtTarget);
		panelsTelemetry.addData("Automatic Transfer Running", transfer.runAutomaticTransfer);


		panelsTelemetry.update();
		panelsTelemetry.update(telemetry);
	}

	public static class Paths {
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
			if (team.equals(Team.BLUE)) {
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

				toSpikeOne = follower
						.pathBuilder()
						.addPath(
								new BezierLine(new Pose(59.440, 17.328), new Pose(46.000, 90.50))
						)
						.setLinearHeadingInterpolation(
								Math.toRadians(294.935),
								Math.toRadians(180)
						)
						.build();

				collectSpikeOne = follower
						.pathBuilder()
						.addPath(
								new BezierLine(new Pose(46.000, 90.50), new Pose(40.000, 40.50))
						)
						.setConstantHeadingInterpolation(Math.toRadians(90))
						.build();

				toShootSpikeOne = follower
						.pathBuilder()
						.addPath(
								new BezierLine(new Pose(40.000, 40.50), new Pose(59.440, 17.328))
						)
						.setLinearHeadingInterpolation(
								Math.toRadians(90),
								Math.toRadians(294.935)
						)
						.build();

				toSpikeTwo = follower
						.pathBuilder()
						.addPath(
								new BezierLine(new Pose(59.44, 17.328), new Pose(46.000, 65.500))
						)
						.setLinearHeadingInterpolation(
								Math.toRadians(294.935),
								Math.toRadians(180)
						)
						.build();

				collectSpikeTwo = follower
						.pathBuilder()
						.addPath(
								new BezierLine(new Pose(46, 65.500), new Pose(40.000, 65.500))
						)
						.setConstantHeadingInterpolation(Math.toRadians(180))
						.build();

				toShootSpikeTwo = follower
						.pathBuilder()
						.addPath(
								new BezierLine(new Pose(40.000, 65.500), new Pose(59.440, 17.328))
						)
						.setLinearHeadingInterpolation(
								Math.toRadians(180),
								Math.toRadians(294.935)
						)
						.build();

				toSpikeThree = follower
						.pathBuilder()
						.addPath(
								new BezierLine(new Pose(59.44, 17.328), new Pose(46.000, 89.500))
						)
						.setLinearHeadingInterpolation(
								Math.toRadians(294.935),
								Math.toRadians(180)
						)
						.build();

				toCollectSpikeThree = follower
						.pathBuilder()
						.addPath(
								new BezierLine(new Pose(46.000, 89.500), new Pose(17.000, 89.500))
						)
						.setConstantHeadingInterpolation(Math.toRadians(180))
						.build();

				toShootSpikeThree = follower.pathBuilder().addPath(
								new BezierCurve(
										new Pose(17.000, 89.500),
										new Pose(52.449, 88.976),
										new Pose(15.351, 84.391),
										new Pose(59.000, 17.000)))

						.setLinearHeadingInterpolation(
								Math.toRadians(180),
								Math.toRadians(294.935)
						)
						.build();
			} else if (team.equals(Team.RED)) {
				shootPreload = follower.pathBuilder().addPath(
								new BezierLine(
										new Pose(81.000, 9.000),

										new Pose(83.000, 11)
								)
						).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-114.14))

						.build();

				toSpikeOne = follower.pathBuilder().addPath(
								new BezierLine(
										new Pose(83.000, 17.328),

										new Pose(98.000, 35.000)
								)
						).setLinearHeadingInterpolation(Math.toRadians(-114.14), Math.toRadians(0))

						.build();

				collectSpikeOne = follower.pathBuilder().addPath(
								new BezierLine(
										new Pose(98.000, 35),
										new Pose(143.000, 35)
								)
						).setConstantHeadingInterpolation(Math.toRadians(0))
						.build();

				toShootSpikeOne = follower.pathBuilder().addPath(
								new BezierLine(
										new Pose(143.000, 35),

										new Pose(83.000, 11.000)
								)
						).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-114.14))

						.build();


				toSpikeTwo = follower.pathBuilder().addPath(
								new BezierLine(
										new Pose(83.000, 11.000),

										new Pose(98.000, 60.000)
								)
						).setLinearHeadingInterpolation(Math.toRadians(-114.14), Math.toRadians(0))
						.build();

				collectSpikeTwo = follower.pathBuilder().addPath(
								new BezierLine(
										new Pose(98.000, 60.000),

										new Pose(143.000, 60.000)
								)
						).setConstantHeadingInterpolation(Math.toRadians(0))
						.build();

				toShootSpikeTwo = follower.pathBuilder().addPath(
								new BezierLine(
										new Pose(143.000, 60.000),

										new Pose(83.000, 11.000)
								)
						).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-114.14))
						.build();

				toParkSpikeTwo = follower
						.pathBuilder()
						.addPath(
								new BezierLine(new Pose(83, 11), new Pose(105, 34))
						)
						.setLinearHeadingInterpolation(
								Math.toRadians(-114.14),
								Math.toRadians(0)
						)
						.build();

				toSpikeThree = follower.pathBuilder().addPath(
								new BezierLine(
										new Pose(83.000, 11.000),

										new Pose(98.000, 84.000)
								)
						).setLinearHeadingInterpolation(Math.toRadians(-114.14), Math.toRadians(0))

						.build();

				toCollectSpikeThree = follower.pathBuilder().addPath(
								new BezierLine(
										new Pose(98.000, 84.000),

										new Pose(129.000, 84.000)
								)
						).setConstantHeadingInterpolation(Math.toRadians(0))

						.build();

				toShootSpikeThree = follower.pathBuilder().addPath(
								new BezierCurve(
										new Pose(129.000, 89.500),
										new Pose(61.063, 91.416),
										new Pose(85.000, 17.000)
								)
						).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-114.14))

						.build();
			}
		}
	}
}