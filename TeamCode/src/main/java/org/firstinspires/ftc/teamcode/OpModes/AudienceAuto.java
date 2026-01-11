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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
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
	
	public Servo rgbServo;
	
	ElapsedTime timer = new ElapsedTime();
	
	double indicatorValue() {
		double x = timer.seconds();
		double hz = 1;
		if (hardwareMap.voltageSensor.iterator().next().getVoltage() >= 13.5){
			hz = 2.5;
		} else if (hardwareMap.voltageSensor.iterator().next().getVoltage() <= 13.5 && hardwareMap.voltageSensor.iterator().next().getVoltage() >= 12) {
			hz = 1;
		}else if (hardwareMap.voltageSensor.iterator().next().getVoltage() <= 11 ) {
			hz = 0.5;
		}else{
			hz = 0.5;
		}
		int state = Math.floorMod((int) Math.floor(x*hz), 2);
		return 0.23 * state + 0.388;
	}

	@Override
	public void init() {
		panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

		follower = Constants.createFollower(hardwareMap);
		follower.setStartingPose(new Pose(63.000, 9, Math.toRadians(270)));

		scheduler = CommandScheduler.getInstance();

		intake = new Intake(hardwareMap);
		shooter = new Shooter(hardwareMap);
		spindexer = new Spindexer(hardwareMap);
		transfer = new Transfer(hardwareMap);
		paths = new Paths(follower);
		
		rgbServo = hardwareMap.get(Servo.class, "rgbIndicator");

		panelsTelemetry.debug("Status", "Initialized");
		panelsTelemetry.update(telemetry);
	}

	@Override
	public void start() {
		timer.startTime();
		scheduler.schedule(
				new SequentialCommandGroup(
						transfer.TransferIn(),
						new FollowPathCommand(follower, paths.shootPreload),
						shooter.SetTarget(Shooter.AUDIENCE_RPM, Shooter.AUDIENCE_RPM), // Start spinning up the shooter
						new ParallelCommandGroup(
								spindexer.DirectPower(0.3),
								transfer.IntakeDoorOut(),
								intake.Slow()
						),
						shooter.WaitForTarget().withTimeout(500), // Wait for the shooter to finish spinning up
						transfer.TransferOut(), // Allow artifacts to leave the spindexer and be shot by the shooter
						new WaitCommand(2500), // Wait 2.5 seconds to allow for all three artifacts to be shot
						transfer.IntakeDoorStop(),
						new ParallelCommandGroup( // Turn off the transfers and shooter
								spindexer.DirectPower(0),
								shooter.SetTarget(0, 0),
								intake.Stop(),
								transfer.TransferStop()
						),
						new FollowPathCommand(follower, paths.toSpikeOne),
						transfer.TransferIn(),
						new ParallelCommandGroup(
							spindexer.DirectPower(0.3),
							transfer.IntakeDoorOut(),
							intake.In()
						),
						new WaitCommand(30), // 30 millisecond wait
						new FollowPathCommand(follower, paths.collectSpikeOne),
						new WaitCommand(30),
						transfer.TransferStop(),
						new ParallelCommandGroup(
								spindexer.DirectPower(0),
								transfer.IntakeDoorStop(),
								intake.SlowOut()
						),
						new FollowPathCommand(follower, paths.toShootSpikeOne),
						shooter.SetTarget(Shooter.AUDIENCE_RPM, Shooter.AUDIENCE_RPM), // Start spinning up the shooter
						shooter.WaitForTarget().withTimeout(500), // Wait for the shooter to finish spinning up
						transfer.IntakeDoorOut(),
						new ParallelCommandGroup(
							transfer.TransferOut(),
							intake.Slow(),
							spindexer.DirectPower(0.3)		
						),
						new WaitCommand(2500), // Wait 2.5 seconds to allow for all three artifacts to be shot
						
						// Spike 2
						transfer.IntakeDoorStop(),
						new ParallelCommandGroup( // Turn off the transfers and shooter
								spindexer.DirectPower(0),
								shooter.SetTarget(0, 0),
								intake.Stop(),
								transfer.TransferStop()
						),
						new FollowPathCommand(follower, paths.toSpikeTwo),
						transfer.TransferIn(),
						new ParallelCommandGroup(
								spindexer.DirectPower(0.3),
								transfer.IntakeDoorOut(),
								intake.In()
						),
						new WaitCommand(30), // 30 millisecond wait
						new FollowPathCommand(follower, paths.collectSpikeTwo),
						new WaitCommand(30),
						transfer.TransferStop(),
						new ParallelCommandGroup(
								spindexer.DirectPower(0),
								transfer.IntakeDoorStop(),
								intake.SlowOut()
						),
						new FollowPathCommand(follower, paths.toShootSpikeTwo),
						shooter.SetTarget(Shooter.AUDIENCE_RPM, Shooter.AUDIENCE_RPM), // Start spinning up the shooter
						shooter.WaitForTarget().withTimeout(500), // Wait for the shooter to finish spinning up
						transfer.IntakeDoorOut(),
						new ParallelCommandGroup(
								transfer.TransferOut(),
								intake.Slow(),
								spindexer.DirectPower(0.3)
						),
						new WaitCommand(2500), // Wait 2.5 seconds to allow for all three artifa

						// Spike the third
						transfer.IntakeDoorStop(),
						new ParallelCommandGroup( // Turn off the transfers and shooter
								spindexer.DirectPower(0),
								shooter.SetTarget(0, 0),
								intake.Stop(),
								transfer.TransferStop()
						),
						new FollowPathCommand(follower, paths.toSpikeThree),
						transfer.TransferIn(),
						new ParallelCommandGroup(
								spindexer.DirectPower(0.3),
								transfer.IntakeDoorOut(),
								intake.In()
						),
						new WaitCommand(30), // 30 millisecond wait
						new FollowPathCommand(follower, paths.toCollectSpikeThree),
						new WaitCommand(30),
						transfer.TransferStop(),
						new ParallelCommandGroup(
								spindexer.DirectPower(0),
								transfer.IntakeDoorStop(),
								intake.SlowOut()
						),
						new FollowPathCommand(follower, paths.toShootSpikeThree),
						shooter.SetTarget(Shooter.AUDIENCE_RPM, Shooter.AUDIENCE_RPM), // Start spinning up the shooter
						shooter.WaitForTarget().withTimeout(500), // Wait for the shooter to finish spinning up
						transfer.IntakeDoorOut(),
						new ParallelCommandGroup(
								transfer.TransferOut(),
								intake.Slow(),
								spindexer.DirectPower(0.3)
						),
						new WaitCommand(2500) // Wait 2.5 seconds to allow for all three artifa
				)
		);

	}

	@Override
	public void loop() {
		follower.update();
		scheduler.run();
		rgbServo.setPosition(indicatorValue());
		shooter.periodic();
		


//		// Log values to Panels and Driver Station
//		panelsTelemetry.debug("X", follower.getPose().getX());
//		panelsTelemetry.debug("Y", follower.getPose().getY());
//		panelsTelemetry.debug("Heading", follower.getPose().getHeading());
//		panelsTelemetry.update(telemetry);
	}

	public static class Paths {
		public PathChain shootPreload;
		public PathChain toSpikeOne;
		public PathChain collectSpikeOne;
		public PathChain toShootSpikeOne;
		public PathChain toSpikeTwo;
		public PathChain collectSpikeTwo;
		public PathChain toShootSpikeTwo;
		public PathChain toSpikeThree;
		public PathChain toCollectSpikeThree;
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

			toSpikeOne = follower
					.pathBuilder()
					.addPath(
							new BezierLine(new Pose(59.440, 17.328), new Pose(41.000, 40.50))
					)
					.setLinearHeadingInterpolation(
							Math.toRadians(294.935),
							Math.toRadians(180)
					)
					.build();

			collectSpikeOne = follower
					.pathBuilder()
					.addPath(
							new BezierLine(new Pose(41.000, 40.50), new Pose(9.000, 40.50))
					)
					.setConstantHeadingInterpolation(Math.toRadians(180))
					.build();

			toShootSpikeOne = follower
					.pathBuilder()
					.addPath(
							new BezierLine(new Pose(9.000, 40.50), new Pose(59.440, 17.328))
					)
					.setLinearHeadingInterpolation(
							Math.toRadians(180),
							Math.toRadians(294.935)
					)
					.build();

			toSpikeTwo = follower
					.pathBuilder()
					.addPath(
							new BezierLine(new Pose(59.44, 17.328), new Pose(41.000, 65.500))
					)
					.setLinearHeadingInterpolation(
							Math.toRadians(294.935),
							Math.toRadians(180)
					)
					.build();

			collectSpikeTwo = follower
					.pathBuilder()
					.addPath(
							new BezierLine(new Pose(41.000, 65.500), new Pose(9.000, 65.500))
					)
					.setConstantHeadingInterpolation(Math.toRadians(180))
					.build();

			toShootSpikeTwo = follower
					.pathBuilder()
					.addPath(
							new BezierLine(new Pose(9.000, 65.500), new Pose(59.440, 17.328))
					)
					.setLinearHeadingInterpolation(
							Math.toRadians(180),
							Math.toRadians(294.935)
					)
					.build();
			toSpikeThree = follower
					.pathBuilder()
					.addPath(
							new BezierLine(new Pose(59.44, 17.328), new Pose(41.000, 89.500))
					)
					.setLinearHeadingInterpolation(
							Math.toRadians(294.935),
							Math.toRadians(180)
					)
					.build();

			toCollectSpikeThree = follower
					.pathBuilder()
					.addPath(
							new BezierLine(new Pose(41.000, 89.500), new Pose(15.000, 89.500))
					)
					.setConstantHeadingInterpolation(Math.toRadians(180))
					.build();

			toShootSpikeThree = follower
					.pathBuilder()
					.addPath(
							new BezierLine(new Pose(15.000, 89.500), new Pose(59.000, 17.328))
					)
					.setLinearHeadingInterpolation(
							Math.toRadians(180),
							Math.toRadians(294.935)
					)
					.build();
		}
	}
}