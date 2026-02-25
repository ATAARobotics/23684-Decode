//package org.firstinspires.ftc.teamcode.OpModes.Auto.Goal;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.seattlesolvers.solverslib.command.CommandScheduler;
//import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
//import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
//import com.seattlesolvers.solverslib.command.WaitCommand;
//import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
//
//import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.Subsystem.Intake;
//import org.firstinspires.ftc.teamcode.Subsystem.Shooter;
//import org.firstinspires.ftc.teamcode.Subsystem.Spindexer;
//import org.firstinspires.ftc.teamcode.Subsystem.Transfer;
//import org.firstinspires.ftc.teamcode.Utils.RobotPosition;
//import org.firstinspires.ftc.teamcode.Utils.ShootArtifacts;
//import org.firstinspires.ftc.teamcode.Utils.Team;
//
//@Configurable
//public abstract class GoalAuto extends OpMode {
//	public Follower follower;
//	private CommandScheduler scheduler;
//	private Intake intake;
//	private Shooter shooter;
//	private Spindexer spindexer;
//	private Transfer transfer;
//	private TelemetryManager panelsTelemetry;
//	private Paths paths;
//
//	protected abstract Pose getStartingPose();
//
//	protected abstract Team getTeam();
//
//	@Override
//	public void stop() {
//		if (follower != null) {
//			RobotPosition.robotPose = follower.getPose();
//			RobotPosition.isPoseSet = true;
//		}
//		CommandScheduler.getInstance().reset();
//	}
//
//	@Override
//	public void init() {
//		panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
//
//		follower = Constants.createFollower(hardwareMap);
//		follower.setStartingPose(getStartingPose());
//
//		scheduler = CommandScheduler.getInstance();
//		intake = new Intake(hardwareMap);
//		shooter = new Shooter(hardwareMap);
//		spindexer = new Spindexer(hardwareMap);
//		transfer = new Transfer(hardwareMap);
//
//		paths = new Paths(follower, getTeam());
//
//		panelsTelemetry.debug("Status", "Initialized");
//		panelsTelemetry.update(telemetry);
//	}
//
//	@Override
//	public void start() {
//		scheduler.schedule(
//				new SequentialCommandGroup(
//						new FollowPathCommand(follower, paths.shootPreload),
//						transfer.SetAutomaticTransfer(true),
//						new ShootArtifacts(shooter, spindexer, transfer, intake),
//						transfer.SetAutomaticTransfer(false),
//
//						new ParallelCommandGroup(
//								shooter.SetTarget(0, 0),
//								transfer.TransferStop(),
//								new FollowPathCommand(follower, paths.toSpikeOne)
//						),
//						new WaitCommand(30),
//						new FollowPathCommand(follower, paths.collectSpikeOne)
////						new ParallelCommandGroup(
////								new FollowPathCommand(follower, paths.toShootSpikeOne),
////								intake.Out(),
////								shooter.SetTarget(Shooter.GOAL_RPM_UPPER, Shooter.GOAL_RPM_LOWER),
////								shooter.WaitForTarget()
////						),
////						transfer.TransferOut(),
////						new ParallelCommandGroup(
////								shooter.SetTarget(0, 0),
////								intake.In(),
////								transfer.TransferIn(),
////								new FollowPathCommand(follower, paths.toSpikeTwo)
////						),
////						new WaitCommand(30),
////						new FollowPathCommand(follower, paths.collectSpikeTwo),
////						new ParallelCommandGroup(
////								new FollowPathCommand(follower, paths.toShootSpikeTwo),
////								intake.Out(),
////								shooter.SetTarget(Shooter.GOAL_RPM_UPPER, Shooter.GOAL_RPM_LOWER),
////								shooter.WaitForTarget()
////						),
////						transfer.TransferOut(),
////						new ParallelCommandGroup(
////								shooter.SetTarget(0, 0),
////								intake.In(),
////								transfer.TransferIn(),
////								new FollowPathCommand(follower, paths.toSpikeThree)
////						),
////						new WaitCommand(30),
////						new FollowPathCommand(follower, paths.collectSpikeThree),
////						new ParallelCommandGroup(
////								new FollowPathCommand(follower, paths.toShootSpikeThree),
////								intake.Out(),
////								shooter.SetTarget(Shooter.GOAL_RPM_UPPER, Shooter.GOAL_RPM_LOWER),
////								shooter.WaitForTarget()
////						),
////						transfer.TransferOut()
//				)
//		);
//	}
//
//	@Override
//	public void loop() {
//		follower.update();
//		scheduler.run();
//
//		panelsTelemetry.debug("X", follower.getPose().getX());
//		panelsTelemetry.debug("Y", follower.getPose().getY());
//		panelsTelemetry.debug("Heading", follower.getPose().getHeading());
//		panelsTelemetry.update(telemetry);
//	}
//
//	public static class Paths {
//		public PathChain shootPreload;
//		public PathChain toSpikeOne;
//		public PathChain collectSpikeOne;
//		public PathChain toShootSpikeOne;
//		public PathChain toSpikeTwo;
//		public PathChain collectSpikeTwo;
//		public PathChain toShootSpikeTwo;
//		public PathChain toSpikeThree;
//		public PathChain collectSpikeThree;
//		public PathChain toShootSpikeThree;
//
//		public Paths(Follower follower, Team team) {
//			if (team.equals(Team.BLUE)) {
//				shootPreload = follower.pathBuilder().addPath(
//								new BezierLine(
//										new Pose(21.307, 124.098),
//
//										new Pose(59.566, 83.717)
//								)
//						).setLinearHeadingInterpolation(Math.toRadians(324), Math.toRadians(315))
//
//						.build();
//
//				toSpikeOne = follower.pathBuilder().addPath(
//								new BezierLine(
//										new Pose(59.566, 83.717),
//
//										new Pose(58.771, 61.346)
//								)
//						).setConstantHeadingInterpolation(Math.toRadians(315))
//
//						.build();
//			} else if (team.equals(Team.RED)) {
//				shootPreload = follower.pathBuilder().addPath(
//								new BezierLine(
//										new Pose(122.693, 124.098),
//
//										new Pose(84.434, 83.717)
//								)
//						).setLinearHeadingInterpolation(Math.toRadians(-144), Math.toRadians(-135))
//
//						.build();
//				toSpikeOne = follower.pathBuilder().addPath(
//								new BezierLine(
//										new Pose(84.434, 83.717),
//
//										new Pose(84.141, 61.254)
//								)
//						).setConstantHeadingInterpolation(Math.toRadians(-135))
//
//						.build();
//			}
//		}
//	}
//}
