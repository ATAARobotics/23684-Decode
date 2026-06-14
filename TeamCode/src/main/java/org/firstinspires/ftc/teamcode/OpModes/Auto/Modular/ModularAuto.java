package org.firstinspires.ftc.teamcode.OpModes.Auto.Modular;

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
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.Gate;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;
import org.firstinspires.ftc.teamcode.Subsystem.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystem.Transfer;
import org.firstinspires.ftc.teamcode.Utils.RobotPosition;
import org.firstinspires.ftc.teamcode.Utils.ShootArtifacts;
import org.firstinspires.ftc.teamcode.Utils.Team;

import java.util.ArrayList;
import java.util.List;

@Configurable
public abstract class ModularAuto extends OpMode {
	public static int COLLECTION_WAIT = 20;
	public static int HUMAN_PLAYER_COLLECTION_WAIT = 30;

	protected Follower follower;
	protected CommandScheduler scheduler;
	protected Intake intake;
	protected Shooter shooter;
	protected Spindexer spindexer;
	protected Transfer transfer;

	protected Gate gate;
	protected TelemetryManager panelsTelemetry;

	private static class RouteItem {
		RouteStep step;
		int waitMs;
		boolean isWait;

		RouteItem(RouteStep step) {
			this.step = step;
			this.isWait = false;
		}

		RouteItem(int waitMs) {
			this.waitMs = waitMs;
			this.isWait = true;
		}
	}

	private final List<RouteItem> route = new ArrayList<>();
	private Pose currentExpectedPose;
	private String currentStepName = "INITIALIZING";

	@Override
	public void init() {
		panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

		follower = Constants.createFollower(hardwareMap);
		follower.setStartingPose(getStartingPose());
		currentExpectedPose = getStartingPose();

		scheduler = CommandScheduler.getInstance();
		scheduler.reset();
		scheduler.setBulkReading(hardwareMap, LynxModule.BulkCachingMode.AUTO);

		intake = new Intake(hardwareMap);
		shooter = new Shooter(hardwareMap);
		spindexer = new Spindexer(hardwareMap);
		transfer = new Transfer(hardwareMap);
		transfer.setShooter(shooter);
		transfer.setSpindexer(spindexer);
		gate = new Gate(hardwareMap);

		setRoute();

		panelsTelemetry.debug("Status", "Modular Auto Initialized");
		panelsTelemetry.update(telemetry);
	}

	protected abstract Pose getStartingPose();

	protected abstract Team getTeam();

	protected abstract void setRoute();

	protected void addStep(RouteStep step) {
		route.add(new RouteItem(step));
	}

	protected void addStep(int waitMs) {
		route.add(new RouteItem(waitMs));
	}

	@Override
	public void start() {
		spindexer.zeroSpindexer();

		SequentialCommandGroup fullAuto = new SequentialCommandGroup();

		for (RouteItem item : route) {
			if (item.isWait) {
				fullAuto.addCommands(
						new InstantCommand(() -> currentStepName = "WAITING (" + item.waitMs + "ms)"),
						new WaitCommand(item.waitMs)
				);
			} else {
				fullAuto.addCommands(
						new InstantCommand(() -> currentStepName = item.step.name()),
						getCommandForStep(item.step)
				);
			}
		}

		scheduler.schedule(fullAuto);
		scheduler.run();
	}

	private Command getCommandForStep(RouteStep step) {
		Team team = getTeam();
		Pose shootPose = PoseDatabase.getShootPose(team);

		switch (step) {
			case SHOOT_PRELOAD:
				PathChain preloadPath = follower.pathBuilder()
						.addPath(new BezierLine(currentExpectedPose, shootPose))
						.setLinearHeadingInterpolation(currentExpectedPose.getHeading(), shootPose.getHeading())
						.build();
				currentExpectedPose = shootPose;
				return new SequentialCommandGroup(
						new ParallelCommandGroup(
								new FollowPathCommand(follower, preloadPath),
								shooter.SetTarget(Shooter.AUDIENCE_RPM, Shooter.AUDIENCE_RPM)

						),
						transfer.IntakeDoorOut(),
						getShootSequence(1050)
				);

			case COLLECT_SPIKE_1:
				return getCollectSpikeCommand(1, COLLECTION_WAIT);

			case COLLECT_SPIKE_2:
				return getCollectSpikeCommand(2, COLLECTION_WAIT - 300);

			case COLLECT_SPIKE_3:
				return getCollectSpikeCommand(3, COLLECTION_WAIT);

			case COLLECT_HUMAN_PLAYER:
				return getCollectHumanPlayerCommand();

			case COLLECT_HUMAN_PLAYER_WIGGLE:
				return getCollectHumanPlayerCommandWithWiggle(false);

			case COLLECT_HUMAN_PLAYER_CLOSE_WIGGLE:
				return getCollectHumanPlayerCommandWithWiggle(true);

			case SHOOT:
				return getShootStepCommand(1000);

			case SHOOT_LONG_PRESPIN:
				return getShootStepCommand(2000);

			case PARK:
				Pose parkPose = team == Team.BLUE ? PoseDatabase.BLUE_PARK : PoseDatabase.RED_PARK;
				PathChain parkPath = follower.pathBuilder()
						.addPath(new BezierLine(currentExpectedPose, parkPose))
						.setLinearHeadingInterpolation(currentExpectedPose.getHeading(), parkPose.getHeading())
						.build();
				currentExpectedPose = parkPose;
				return new FollowPathCommand(follower, parkPath);

			case WAIT:
				return new WaitCommand(0); // Handled by item.isWait logic, but here for completeness

			default:
				return new WaitCommand(0);
		}
	}

	private Command getCollectSpikeCommand(int spikeNum, int collectionWait) {
		Team team = getTeam();
		Pose intermediate, collect;
		if (team == Team.BLUE) {
			if (spikeNum == 1) {
				intermediate = PoseDatabase.BLUE_SPIKE_1_INTERMEDIATE;
				collect = PoseDatabase.BLUE_SPIKE_1_COLLECT;
			} else if (spikeNum == 2) {
				intermediate = PoseDatabase.BLUE_SPIKE_2_INTERMEDIATE;
				collect = PoseDatabase.BLUE_SPIKE_2_COLLECT;
			} else {
				intermediate = PoseDatabase.BLUE_SPIKE_3_INTERMEDIATE;
				collect = PoseDatabase.BLUE_SPIKE_3_COLLECT;
			}
		} else {
			if (spikeNum == 1) {
				intermediate = PoseDatabase.RED_SPIKE_1_INTERMEDIATE;
				collect = PoseDatabase.RED_SPIKE_1_COLLECT;
			} else if (spikeNum == 2) {
				intermediate = PoseDatabase.RED_SPIKE_2_INTERMEDIATE;
				collect = PoseDatabase.RED_SPIKE_2_COLLECT;
			} else {
				intermediate = PoseDatabase.RED_SPIKE_3_INTERMEDIATE;
				collect = PoseDatabase.RED_SPIKE_3_COLLECT;
			}
		}

		PathChain toSpike = follower.pathBuilder()
				.addPath(new BezierLine(currentExpectedPose, intermediate))
				.setLinearHeadingInterpolation(currentExpectedPose.getHeading(), intermediate.getHeading())
				.build();
		toSpike.setDecelerationType(PathChain.DecelerationType.NONE);

		PathChain forwardPath = follower.pathBuilder()
				.addPath(new BezierLine(intermediate, collect))
				.setConstantHeadingInterpolation(collect.getHeading())
				.build();

		forwardPath.setDecelerationType(PathChain.DecelerationType.NONE);

		currentExpectedPose = collect;

		SequentialCommandGroup command = new SequentialCommandGroup(
				new ParallelCommandGroup(
						new FollowPathCommand(follower, toSpike),
						new SequentialCommandGroup(
								transfer.TransferIn(),
								intake.In(),
								spindexer.DirectPower(1),
								transfer.IntakeDoorOut()
						)
				),
				new FollowPathCommand(follower, forwardPath),
				new WaitCommand(collectionWait),
				transfer.TransferStop()
		);

		return command;
	}

	private Command getCollectHumanPlayerCommand() {
		Team team = getTeam();
		Pose intermediate = (team == Team.BLUE) ? PoseDatabase.BLUE_HUMAN_PLAYER_INTERMEDIATE : PoseDatabase.RED_HUMAN_PLAYER_INTERMEDIATE;
		Pose collect = (team == Team.BLUE) ? PoseDatabase.BLUE_HUMAN_PLAYER_COLLECT : PoseDatabase.RED_HUMAN_PLAYER_COLLECT;

		PathChain toHP = follower.pathBuilder()
				.addPath(new BezierLine(currentExpectedPose, intermediate))
				.setLinearHeadingInterpolation(currentExpectedPose.getHeading(), intermediate.getHeading())
				.build();
		toHP.setDecelerationType(PathChain.DecelerationType.NONE);

		PathChain forwardPath = follower.pathBuilder()
				.addPath(new BezierLine(intermediate, collect))
				.setConstantHeadingInterpolation(collect.getHeading())
				.build();

		forwardPath.setDecelerationType(PathChain.DecelerationType.NONE);

		currentExpectedPose = collect;

		return new SequentialCommandGroup(
				new ParallelCommandGroup(
						new FollowPathCommand(follower, toHP).setGlobalMaxPower(0.9),
						new SequentialCommandGroup(
								new WaitCommand(100),
								transfer.TransferIn(),
								intake.In(),
								spindexer.DirectPower(1),
								transfer.IntakeDoorOut()
						)
				),
				new ParallelCommandGroup(
						new FollowPathCommand(follower, forwardPath).setGlobalMaxPower(1),
						new SequentialCommandGroup(
								transfer.TransferIn(),
								intake.In(),
								spindexer.DirectPower(1),
								transfer.IntakeDoorOut()
						)
				),
				new WaitCommand(HUMAN_PLAYER_COLLECTION_WAIT),
				transfer.TransferStop()
		);
	}

	private Command getCollectHumanPlayerCommandWithWiggle(boolean close) {
		Team team = getTeam();
		Pose intermediate = (team == Team.BLUE) ? PoseDatabase.BLUE_HUMAN_PLAYER_INTERMEDIATE : PoseDatabase.RED_HUMAN_PLAYER_INTERMEDIATE;
		Pose collect;
		Pose wiggle;
		if (!close) {
			collect = (team == Team.BLUE) ? PoseDatabase.BLUE_HUMAN_PLAYER_COLLECT : PoseDatabase.RED_HUMAN_PLAYER_COLLECT;
			wiggle = (team == Team.BLUE) ? PoseDatabase.BLUE_HUMAN_PLAYER_COLLECT_WIGGLE : PoseDatabase.RED_HUMAN_PLAYER_COLLECT_WIGGLE;
		} else {
			collect = (team == Team.BLUE) ? PoseDatabase.BLUE_HUMAN_PLAYER_COLLECT : PoseDatabase.RED_HUMAN_PLAYER_COLLECT_CLOSE;
			wiggle = (team == Team.BLUE) ? PoseDatabase.BLUE_HUMAN_PLAYER_COLLECT_WIGGLE : PoseDatabase.RED_HUMAN_PLAYER_COLLECT_WIGGLE_CLOSE;

		}


		PathChain toHP = follower.pathBuilder()
				.addPath(new BezierLine(currentExpectedPose, intermediate))
				.setLinearHeadingInterpolation(currentExpectedPose.getHeading(), intermediate.getHeading())
				.build();

		toHP.setDecelerationType(PathChain.DecelerationType.NONE);

		PathChain forwardPath = follower.pathBuilder()
				.addPath(new BezierLine(intermediate, collect))
				.setConstantHeadingInterpolation(collect.getHeading())
				.build();

		forwardPath.setDecelerationType(PathChain.DecelerationType.NONE);

		double wiggleOffset = (team == Team.BLUE) ? 1.5 : -1.5;
		Pose shallowWiggle = new Pose(wiggle.getX() + wiggleOffset, wiggle.getY(), wiggle.getHeading());
		PathChain wigglePath = follower.pathBuilder()
				.addPath(new BezierLine(wiggle, shallowWiggle))
				.setConstantHeadingInterpolation(collect.getHeading())
				.addPath(new BezierLine(shallowWiggle, wiggle))
				.setConstantHeadingInterpolation(collect.getHeading())
				.build();

		wigglePath.setDecelerationType(PathChain.DecelerationType.NONE);

		currentExpectedPose = wiggle;

		return new SequentialCommandGroup(
				new ParallelCommandGroup(
						new FollowPathCommand(follower, toHP).setGlobalMaxPower(0.9),
						new SequentialCommandGroup(
								new WaitCommand(100),
								transfer.TransferIn(),
								intake.In(),
								spindexer.DirectPower(1),
								transfer.IntakeDoorOut()
						)
				),
				new ParallelCommandGroup(
						new FollowPathCommand(follower, forwardPath).setGlobalMaxPower(1),
						new SequentialCommandGroup(
								transfer.TransferIn(),
								intake.In(),
								spindexer.DirectPower(1),
								transfer.IntakeDoorOut()
						)
				),
				new WaitCommand(HUMAN_PLAYER_COLLECTION_WAIT),
				new FollowPathCommand(follower, wigglePath).setGlobalMaxPower(1),
				new WaitCommand(HUMAN_PLAYER_COLLECTION_WAIT),
				transfer.TransferStop()
		);
	}

	private Command getShootStepCommand(int prespinWaitMs) {
		Team team = getTeam();
		Pose shootPose = PoseDatabase.getShootPose(team);
		PathChain toShoot;

		// Special handling for Spike 3 return path
		if (currentExpectedPose.equals(PoseDatabase.BLUE_SPIKE_3_COLLECT) && team == Team.BLUE) {
			toShoot = follower.pathBuilder().addPath(
							new BezierCurve(
									new Pose(15.000, 89.500),
									new Pose(52.449, 89.5),
									new Pose(15.351, 84.391),
									shootPose))
					.setLinearHeadingInterpolation(currentExpectedPose.getHeading(), shootPose.getHeading())
					.build();
		} else if (currentExpectedPose.equals(PoseDatabase.RED_SPIKE_3_COLLECT) && team == Team.RED) {
			toShoot = follower.pathBuilder().addPath(
							new BezierCurve(
									new Pose(129.000, 89.500),
									new Pose(61.063, 91.416),
									shootPose)
					).setLinearHeadingInterpolation(currentExpectedPose.getHeading(), shootPose.getHeading())
					.build();
		} else {
			toShoot = follower.pathBuilder()
					.addPath(new BezierLine(currentExpectedPose, shootPose))
					.setLinearHeadingInterpolation(currentExpectedPose.getHeading(), shootPose.getHeading())
					.build();
		}

		toShoot.setDecelerationType(PathChain.DecelerationType.NONE);

		currentExpectedPose = shootPose;

		return new SequentialCommandGroup(
				intake.Slow(),
				spindexer.DirectPower(0),
				new ParallelCommandGroup(
						new FollowPathCommand(follower, toShoot),
						new SequentialCommandGroup(
								new WaitCommand(prespinWaitMs),
								shooter.SetTarget(Shooter.AUDIENCE_RPM, Shooter.AUDIENCE_RPM)
						)
				),
				transfer.IntakeDoorOut(),
				getShootSequence(1050)
		);
	}

	private Command getShootSequence(int waitTime) {
		return new SequentialCommandGroup(
				new ShootArtifacts(shooter, spindexer, transfer, intake, gate, waitTime),
				shooter.SetTarget(0, 0),
				gate.closeGate(),
				spindexer.DirectPower(0),
				intake.Stop(),
				transfer.TransferStop(),
				transfer.IntakeDoorStop()
		);
	}

	@Override
	public void loop() {
		follower.update();
		scheduler.run();
		shooter.periodic();
		transfer.periodic();

		panelsTelemetry.addData("Current Step", currentStepName);

		panelsTelemetry.addLine("=== SHOOTER ===");
		panelsTelemetry.addData("Upper RPM", shooter.upperRPM);
		panelsTelemetry.addData("Lower RPM", shooter.lowerRPM);
		panelsTelemetry.addData("Average RPM", shooter.averageRPM);

		panelsTelemetry.addLine("=== TRANSFER ===");
		panelsTelemetry.addData("Shooter At Target", transfer.reachedAverageTarget);
		panelsTelemetry.addData("Automatic Transfer Running", transfer.runAutomaticTransfer);

		panelsTelemetry.update();
		panelsTelemetry.update(telemetry);
	}

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
}
