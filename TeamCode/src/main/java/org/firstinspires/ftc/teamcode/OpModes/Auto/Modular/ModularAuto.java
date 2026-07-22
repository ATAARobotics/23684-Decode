package org.firstinspires.ftc.teamcode.OpModes.Auto.Modular;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.pedroCommand.TurnToCommand;

import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.BeamBreaker;
import org.firstinspires.ftc.teamcode.Subsystem.Conveyor;
import org.firstinspires.ftc.teamcode.Subsystem.Gate;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;
import org.firstinspires.ftc.teamcode.Subsystem.Transfer;
import org.firstinspires.ftc.teamcode.Utils.Drive;
import org.firstinspires.ftc.teamcode.Utils.Drawing;
import org.firstinspires.ftc.teamcode.Utils.RobotConfig;
import org.firstinspires.ftc.teamcode.Utils.RobotPosition;
import org.firstinspires.ftc.teamcode.Utils.ShootArtifacts;
import org.firstinspires.ftc.teamcode.Utils.Team;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

@Configurable
public abstract class ModularAuto extends OpMode {
	public static int COLLECTION_WAIT = 1;
	public static int HUMAN_PLAYER_COLLECTION_WAIT = 500;
	public  int ShootTime = 1050;

	public int ballcount = 0;
	public boolean shouldShoot = false;

	boolean waitedOnce = false;

	protected Follower follower;
	protected CommandScheduler scheduler;
	protected Intake intake;
	protected Shooter shooter;
	protected Conveyor conveyor;
	protected Transfer transfer;

	protected BeamBreaker beamBreaker;

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
		conveyor = new Conveyor(hardwareMap);
		transfer = new Transfer(hardwareMap);
		transfer.setShooter(shooter);
		gate = new Gate(hardwareMap);
		beamBreaker = new BeamBreaker(hardwareMap);

		// Re-fetch scheduler: scheduler.reset() above nulled the singleton, so
		// the subsystems constructed below registered on a fresh instance. Repoint
		// the local reference at that instance so scheduler.run() invokes their
		// periodic() methods.
		scheduler = CommandScheduler.getInstance();

		setRoute();

		// Repoint at the post-reset scheduler singleton so the subsystems registered
		// above drive the auto routine.
		scheduler = CommandScheduler.getInstance();

		if (!RobotConfig.COMPETITION) {
			panelsTelemetry.debug("Status", "Modular Auto Initialized");
			panelsTelemetry.update(telemetry);
		}
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

		scheduler.schedule(new SequentialCommandGroup(
				fullAuto.withTimeout(29500),
				getCommandForStep(RouteStep.PARK)
		));
		scheduler.run();
	}

	private Command getCommandForStep(RouteStep step) {
		Team team = getTeam();
		Pose shootPose = PoseDatabase.getShootPose(team);

		switch (step) {
			case SHOOT_PRELOAD:
				PathChain preloadPath = follower.pathBuilder()
						.addPath(new BezierLine(currentExpectedPose, shootPose))
						.setLinearHeadingInterpolation(currentExpectedPose.getHeading(), shootPose.getHeading(),0.1)
						.build();
				currentExpectedPose = shootPose;
				return new SequentialCommandGroup(
						gate.closeGate(),
						new ParallelCommandGroup(
								new FollowPathCommand(follower, preloadPath),
								shooter.SetTarget(Shooter.AUDIENCE_RPM_UPPER, Shooter.AUDIENCE_RPM_LOWER)

						),
						transfer.IntakeDoorOut(),
						getShootSequence(700)
				);

			case COLLECT_SPIKE_1:
				return getCollectSpikeCommand(1, COLLECTION_WAIT);

			case COLLECT_SPIKE_2:
				return getCollectSpikeCommand(2, COLLECTION_WAIT);

			case COLLECT_SPIKE_3:
				return getCollectSpikeCommand(3, COLLECTION_WAIT);

			case COLLECT_HUMAN_PLAYER:
				return getCollectHumanPlayerCommand();

			case SHOOT:
				return getShootStepCommand(1000);

			case SHOOT_LONG_PRESPIN:
				return getShootStepCommand(2000);

			case SHOOT_WITH_BEAMBREAKER:
				return getShootWithBeamBreakerStepCommand(1000);

			case PARK:
				Pose parkPose = team == Team.BLUE ? PoseDatabase.BLUE_PARK : PoseDatabase.RED_PARK;
				 Supplier<PathChain> parkPath = () -> follower.pathBuilder()
						.addPath(new BezierLine(follower.getPose(), parkPose))
						.setLinearHeadingInterpolation(follower.getPose().getHeading(), parkPose.getHeading())
						.build();
				currentExpectedPose = parkPose;
				return new ParallelCommandGroup(
						new FollowPathCommand(follower, parkPath.get()),
						conveyor.Stop(),
						intake.Stop(),
						transfer.TransferStop(),
						shooter.SetTarget(0, 0),
						gate.closeGate()

				);

			default:
				throw new IllegalStateException("Unhandled route step: " + step);
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

		PathChain toSpike;

		if (spikeNum == 1){
			toSpike = follower.pathBuilder()
					.addPath(new BezierLine(currentExpectedPose, intermediate))
					.setHeadingInterpolation(
							HeadingInterpolator.piecewise(
									new HeadingInterpolator.PiecewiseNode(
											0,
											.25,
											HeadingInterpolator.tangent.reverse()
									),
									new HeadingInterpolator.PiecewiseNode(
											.25,
											1,
											HeadingInterpolator.linear(currentExpectedPose.getHeading(), intermediate.getHeading(),0.2)
									)
							))
					.setHeadingConstraint(Math.toDegrees(5))
					.setTValueConstraint(0.99)
					.build();
			toSpike.setDecelerationType(PathChain.DecelerationType.NONE);
		} else {
			toSpike = follower.pathBuilder()
					.addPath(new BezierLine(currentExpectedPose, intermediate))
					.setLinearHeadingInterpolation(currentExpectedPose.getHeading(), intermediate.getHeading(),0.2).setHeadingInterpolation(
							HeadingInterpolator.piecewise(
									new HeadingInterpolator.PiecewiseNode(
											0,
											.1,
											HeadingInterpolator.constant(currentExpectedPose.getHeading())
									),
									new HeadingInterpolator.PiecewiseNode(
											.1,
											.5,
											HeadingInterpolator.tangent.reverse()
									),
									new HeadingInterpolator.PiecewiseNode(
											.5,
											1,
											HeadingInterpolator.linear(follower.getPose().getHeading(), intermediate.getHeading(),0.1)
									)
							))
					.setHeadingConstraint(Math.toDegrees(5))
					.setTValueConstraint(0.99)
					.build();
			toSpike.setDecelerationType(PathChain.DecelerationType.NONE);
		}

		PathChain forwardPath = follower.pathBuilder()
				.addPath(new BezierLine(intermediate, collect))
				.setConstantHeadingInterpolation(collect.getHeading())
				.build();

		forwardPath.setDecelerationType(PathChain.DecelerationType.NONE);

		currentExpectedPose = collect;

		SequentialCommandGroup command = new SequentialCommandGroup(
				//new ParallelCommandGroup(
						new FollowPathCommand(follower, toSpike).interruptOn(()-> follower.getCurrentTValue() >= 0.9),
						new SequentialCommandGroup(
								transfer.TransferIn(),
								intake.In(),
								conveyor.In(),
								transfer.IntakeDoorOut()
						),
				//),
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

		Pose collectSpikeone = (team == Team.BLUE) ? PoseDatabase.BLUE_SPIKE_1_COLLECT : PoseDatabase.RED_SPIKE_1_COLLECT;
		Pose spikeOne = (team == Team.BLUE) ? PoseDatabase.BLUE_HUMAN_PLAYER_COLLECT_FARSIDE : PoseDatabase.RED_HUMAN_PLAYER_COLLECT_FARSIDE;

		PathChain toHP = follower.pathBuilder()
				.addPath(new BezierLine(currentExpectedPose, intermediate))
				.setHeadingInterpolation(HeadingInterpolator.linear(currentExpectedPose.getHeading(), intermediate.getHeading(),0.2))
				.addPath(new BezierLine(intermediate, collect))
				.setConstantHeadingInterpolation(collect.getHeading())
				.build();
		toHP.setDecelerationType(PathChain.DecelerationType.NONE);

		PathChain forwardPath = follower.pathBuilder()
				.addPath(new BezierLine(intermediate, collect))
				.setConstantHeadingInterpolation(collect.getHeading())
				.build();

		forwardPath.setDecelerationType(PathChain.DecelerationType.NONE);

		PathChain awayfromHp = follower.pathBuilder()
				.addPath(new BezierLine(collect, intermediate))
				.setConstantHeadingInterpolation(collect.getHeading())
				.build();

		PathChain toSpikeOne = follower.pathBuilder()
				.addPath(new BezierLine(intermediate,spikeOne))
				.setConstantHeadingInterpolation(collect.getHeading())
				.build();

		toSpikeOne.setDecelerationType(PathChain.DecelerationType.NONE);

		PathChain collectSpikeOne = follower.pathBuilder()
				.addPath(new BezierLine(spikeOne,collectSpikeone))
				.setConstantHeadingInterpolation(collect.getHeading())
				.build();

		collectSpikeOne.setDecelerationType(PathChain.DecelerationType.NONE);

		PathChain leaveSpikeOne = follower.pathBuilder()
				.addPath(new BezierLine(collectSpikeone,spikeOne))
				.setConstantHeadingInterpolation(collect.getHeading())
				.build();


		PathChain backtoHp = follower.pathBuilder()
				.addPath(new BezierLine(spikeOne,intermediate))
				.setConstantHeadingInterpolation(collect.getHeading())
				.build();

		backtoHp.setDecelerationType(PathChain.DecelerationType.NONE);


		currentExpectedPose = collect;

		return new SequentialCommandGroup(

				new ParallelCommandGroup(
						new FollowPathCommand(follower,toHP).setGlobalMaxPower(1),
						new SequentialCommandGroup(
								intake.In(),
								conveyor.In(),
								transfer.IntakeDoorOut()
						)
				),
				new RepeatCommand(
					new SequentialCommandGroup(
							new InstantCommand(()-> waitedOnce = false),

							new InstantCommand(()-> waitedOnce = true),

							new FollowPathCommand(follower,awayfromHp),

							new InstantCommand(()-> waitedOnce = false),

							new FollowPathCommand(follower,toSpikeOne),
							new FollowPathCommand(follower,collectSpikeOne),
							new WaitCommand(HUMAN_PLAYER_COLLECTION_WAIT),
							new InstantCommand(()-> waitedOnce = true),

							new FollowPathCommand(follower,leaveSpikeOne),
							new FollowPathCommand(follower,backtoHp),

		                    new WaitCommand(HUMAN_PLAYER_COLLECTION_WAIT),

							new FollowPathCommand(follower, forwardPath),
							new WaitCommand(HUMAN_PLAYER_COLLECTION_WAIT)


							), () -> ballcount != 0 //&& waitedOnce
				),
				new InstantCommand(()-> currentExpectedPose = follower.getPose()),
				transfer.TransferStop()
		);
	}

	private Command getShootStepCommand(int prespinWaitMs) {
		Team team = getTeam();
		Pose shootPose = PoseDatabase.getShootPose(team);
		PathChain toShoot;

		// Special handling for Spike 3 return path
		if (currentExpectedPose.equals(PoseDatabase.BLUE_SPIKE_3_COLLECT) && team == Team.BLUE) {
//			toShoot = follower.pathBuilder().addPath(
//							new BezierCurve(
//									new Pose(15.000, 89.000),
//									new Pose(54.810, 88.063),
//									shootPose))
//					.setBrakingStrength(1)
//					.setHeadingInterpolation(
//							HeadingInterpolator.piecewise(
//									new HeadingInterpolator.PiecewiseNode(
//											0,
//											.8,
//											HeadingInterpolator.tangent.reverse()
//									),
//									new HeadingInterpolator.PiecewiseNode(
//											.8,
//											1,
//											HeadingInterpolator.linear(follower.getPose().getHeading(), shootPose.getHeading(),0.1)
//									)
//							))
//					.build();
			toShoot = follower.pathBuilder().addPath(
							new BezierLine(currentExpectedPose.getPose(), new Pose(PoseDatabase.BLUE_SPIKE_3_INTERMEDIATE.getX() - 4, PoseDatabase.BLUE_SPIKE_3_INTERMEDIATE.getY())))
					.setBrakingStrength(1)
					.setBrakingStart(0.7)
					.setConstantHeadingInterpolation(PoseDatabase.BLUE_SPIKE_3_COLLECT.getHeading())
					.addPath(new BezierLine(new Pose(PoseDatabase.BLUE_SPIKE_3_INTERMEDIATE.getX() - 4, PoseDatabase.BLUE_SPIKE_3_INTERMEDIATE.getY()), shootPose))
					.setHeadingInterpolation(
							HeadingInterpolator.piecewise(
							new HeadingInterpolator.PiecewiseNode(
									.0,
									.8,
									HeadingInterpolator.tangent
							),
							new HeadingInterpolator.PiecewiseNode(
									.8,
									1,
									HeadingInterpolator.linear(follower.getPose().getHeading(), shootPose.getHeading(),0.1)
							)
					))
					.build();
			prespinWaitMs += 400;
		} else if (currentExpectedPose.equals(PoseDatabase.RED_SPIKE_3_COLLECT) && team == Team.RED) {
			toShoot = follower.pathBuilder().addPath(
							new BezierLine(currentExpectedPose.getPose(), new Pose(PoseDatabase.RED_SPIKE_3_INTERMEDIATE.getX() + 4, PoseDatabase.RED_SPIKE_3_INTERMEDIATE.getY())))
					.setBrakingStrength(1)
					.setConstantHeadingInterpolation(PoseDatabase.RED_SPIKE_3_INTERMEDIATE.getHeading())
					.addPath(new BezierLine(new Pose(PoseDatabase.RED_SPIKE_3_INTERMEDIATE.getX() + 4, PoseDatabase.RED_SPIKE_3_INTERMEDIATE.getY()), shootPose))
					.setHeadingInterpolation(
							HeadingInterpolator.piecewise(
									new HeadingInterpolator.PiecewiseNode(
											.0,
											.8,
											HeadingInterpolator.tangent
									),
									new HeadingInterpolator.PiecewiseNode(
											.8,
											1,
											HeadingInterpolator.linear(follower.getPose().getHeading(), shootPose.getHeading(),0.1)
									)
							))
					.build();
			prespinWaitMs += 400;
		}else if  (currentExpectedPose.equals(PoseDatabase.RED_SPIKE_1_COLLECT) || currentExpectedPose.equals(PoseDatabase.BLUE_SPIKE_1_COLLECT)){
			toShoot = follower.pathBuilder()
					.addPath(new BezierLine(currentExpectedPose, shootPose))
					.setBrakingStrength(1)
					.setBrakingStart(0.7)
					.setHeadingInterpolation(HeadingInterpolator.linear(currentExpectedPose.getHeading(), shootPose.getHeading(),0.6))
					.build();
		}else{
			toShoot = follower.pathBuilder()
					.addPath(new BezierLine(currentExpectedPose, shootPose))
					.setBrakingStrength(1)
					.setBrakingStart(0.7)
					.setHeadingInterpolation(
							HeadingInterpolator.piecewise(
									new HeadingInterpolator.PiecewiseNode(
											0,
											.1,
											HeadingInterpolator.constant(currentExpectedPose.getHeading())
									),
									new HeadingInterpolator.PiecewiseNode(
											.1,
											.8,
											HeadingInterpolator.tangent
									),
									new HeadingInterpolator.PiecewiseNode(
											.8,
											1,
											HeadingInterpolator.linear(follower.getPose().getHeading(), shootPose.getHeading(),0.1)
									)
							))
					.build();
		}

		toShoot.setDecelerationType(PathChain.DecelerationType.NONE);

		currentExpectedPose = shootPose;

		return new SequentialCommandGroup(
				intake.In(),
				new ParallelCommandGroup(
						new SequentialCommandGroup(
								new WaitUntilCommand(()-> follower.getCurrentTValue() >= 0.6),
								shooter.SetTarget(Shooter.AUDIENCE_RPM_UPPER,Shooter.AUDIENCE_RPM_LOWER),
								intake.Stop()
						),
						new FollowPathCommand(follower, toShoot),
						new InstantCommand(()-> {
							shouldShoot = ballcount > 0;
							if (ballcount > 1) ShootTime = 1050;
							else ShootTime = 500;
						})

				),
				transfer.IntakeDoorOut(),
				getShootSequence(550),
				new InstantCommand(()-> beamBreaker.resetBallCount())
		);
	}


	private Command getShootWithBeamBreakerStepCommand(int prespinWaitMs) {
		Team team = getTeam();
		Pose shootPose = PoseDatabase.getShootPose(team);
		PathChain toShoot;

		// Special handling for Spike 3 return path
		if (currentExpectedPose.equals(PoseDatabase.BLUE_SPIKE_3_COLLECT) && team == Team.BLUE) {
//			toShoot = follower.pathBuilder().addPath(
//							new BezierCurve(
//									new Pose(15.000, 89.500),
//									new Pose(52.449, 89.5),
//									new Pose(15.351, 84.391),
//									shootPose))
//					.setBrakingStrength(1)
//					.setBrakingStart(0.7)
//					.setHeadingInterpolation(
//							HeadingInterpolator.piecewise(
//									new HeadingInterpolator.PiecewiseNode(
//											0,
//											.8,
//											HeadingInterpolator.tangent.reverse()
//									),
//									new HeadingInterpolator.PiecewiseNode(
//											.8,
//											1,
//											HeadingInterpolator.linear(follower.getPose().getHeading(), shootPose.getHeading(),0.1)
//									)
//							))
//					.build();
			toShoot = follower.pathBuilder()
					.addPath(new BezierLine(currentExpectedPose, shootPose))
					.setBrakingStrength(1)
					.setHeadingInterpolation(
							HeadingInterpolator.piecewise(
									new HeadingInterpolator.PiecewiseNode(
											0,
											.8,
											HeadingInterpolator.tangent.reverse()
									),
									new HeadingInterpolator.PiecewiseNode(
											.8,
											1,
											HeadingInterpolator.linear(follower.getPose().getHeading(), shootPose.getHeading(),0.1)
									)
							))
					.build();
			prespinWaitMs += 400;
		} else if (currentExpectedPose.equals(PoseDatabase.RED_SPIKE_3_COLLECT) && team == Team.RED) {
//			toShoot = follower.pathBuilder().addPath(
//							new BezierCurve(
//									new Pose(129.000, 89.500),
//									new Pose(61.063, 91.416),
//									shootPose)
//					)
//					.setBrakingStrength(1)
//					.setBrakingStart(0.7)
//					.setHeadingInterpolation(
//							HeadingInterpolator.piecewise(
//									new HeadingInterpolator.PiecewiseNode(
//											0,
//											.9,
//											HeadingInterpolator.tangent.reverse()
//									),
//									new HeadingInterpolator.PiecewiseNode(
//											.9,
//											1,
//											HeadingInterpolator.linear(follower.getPose().getHeading(), shootPose.getHeading(),0.1)
//									)
//							))
//					.build();
			toShoot = follower.pathBuilder()
					.addPath(new BezierLine(currentExpectedPose, shootPose))
					.setBrakingStrength(1)
					.setHeadingInterpolation(
							HeadingInterpolator.piecewise(
									new HeadingInterpolator.PiecewiseNode(
											0,
											.8,
											HeadingInterpolator.tangent.reverse()
									),
									new HeadingInterpolator.PiecewiseNode(
											.8,
											1,
											HeadingInterpolator.linear(follower.getPose().getHeading(), shootPose.getHeading(),0.1)
									)
							))
					.build();
			prespinWaitMs += 400;
		} else {
			toShoot = follower.pathBuilder()
					.addPath(new BezierLine(currentExpectedPose, shootPose))
					.setBrakingStrength(1)
					.setHeadingInterpolation(
							HeadingInterpolator.piecewise(
									new HeadingInterpolator.PiecewiseNode(
											0,
											.1,
											HeadingInterpolator.constant(PoseDatabase.BLUE_SPIKE_3_COLLECT.getHeading())
									),
									new HeadingInterpolator.PiecewiseNode(
											.1,
											.5,
											HeadingInterpolator.tangent
									),
									new HeadingInterpolator.PiecewiseNode(
											.5,
											1,
											HeadingInterpolator.linear(follower.getPose().getHeading(), shootPose.getHeading(),0.1)
									)
							))
					.build();
		}

		toShoot.setDecelerationType(PathChain.DecelerationType.NONE);

		currentExpectedPose = shootPose;

		return new SequentialCommandGroup(
				intake.In(),
				new ParallelCommandGroup(
						new FollowPathCommand(follower, toShoot),
						new SequentialCommandGroup(
								new WaitUntilCommand(()-> follower.getCurrentTValue() >= 0.6),
								shooter.SetTarget(Shooter.AUDIENCE_RPM_UPPER,Shooter.AUDIENCE_RPM_LOWER),
								intake.Stop()
						),
						new InstantCommand(()-> {
							shouldShoot = ballcount > 0;
							if (ballcount > 1) ShootTime = 750;
							else ShootTime = 250;
						})
				),
				new TurnToCommand(follower, shootPose.getHeading()),
				transfer.IntakeDoorOut().interruptOn(()-> !shouldShoot),
				new ConditionalCommand(
						getShootSequence(550),
						getShootSequence(100),
						()-> ballcount > 1).interruptOn(()-> !shouldShoot),
				new InstantCommand(()-> beamBreaker.resetBallCount())
		);
	}

	private Command getShootSequence(int waitTime) {
		return new SequentialCommandGroup(
				new ShootArtifacts(shooter, conveyor, transfer, intake, gate, waitTime),
				shooter.SetTarget(0, 0),
				gate.closeGate(),
				conveyor.Stop(),
				intake.Stop(),
				transfer.TransferStop(),
				transfer.IntakeDoorStop()
		);
	}

	@Override
	public void loop() {
		follower.update();
		scheduler.run();
		beamBreaker.update(true);


		ballcount = beamBreaker.getBallCount();


		if (RobotConfig.COMPETITION) return;

		panelsTelemetry.addData("Current Step", currentStepName);
		panelsTelemetry.addData("Location", follower.getPose().toString());
		Drawing.drawRobot(follower.getPose());
		Drawing.sendPacket();

		panelsTelemetry.addLine("=== SHOOTER ===");
		panelsTelemetry.addData("Upper RPM", shooter.upperRPM);
		panelsTelemetry.addData("Lower RPM", shooter.lowerRPM);
		panelsTelemetry.addData("Average RPM", shooter.averageRPM);

		panelsTelemetry.addLine("=== TRANSFER ===");
		panelsTelemetry.addData("Shooter At Target", transfer.reachedAverageTarget);
		panelsTelemetry.addData("Automatic Transfer Running", transfer.runAutomaticTransfer);

		panelsTelemetry.update();
		panelsTelemetry.update(telemetry);

		telemetry.addData("ballcount", beamBreaker.getBallCount());
		telemetry.addData("tvalue",follower.getCurrentTValue());
		telemetry.update();
	}

	@Override
	public void stop() {
		if (follower != null) {
			RobotPosition.robotPose = follower.getPose();
			RobotPosition.isPoseSet = true;
		}
		CommandScheduler.getInstance().reset();
	}
}
