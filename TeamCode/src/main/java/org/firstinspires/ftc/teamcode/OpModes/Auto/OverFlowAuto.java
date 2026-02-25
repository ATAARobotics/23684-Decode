package org.firstinspires.ftc.teamcode.OpModes.Auto;

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
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
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
public abstract class OverFlowAuto extends OpMode {
    // Configurable wait times (in milliseconds)
    public static int SHOOT_DWELL_TIME = 3200; // Time to allow all artifacts to be shot
    public static int SPIKE_COLLECTION_WAIT = 20; // Short wait during spike collection

    public int loopendtime = 0;
    private final ElapsedTime timer = new ElapsedTime();
    public Follower follower;
    public Paths paths;
    private CommandScheduler scheduler;
    private Intake intake;
    private Shooter shooter;
    private Spindexer spindexer;
    private Transfer transfer;

    private Touch touch;
    private TelemetryManager panelsTelemetry;

    @Override
    public void stop() {
        if (follower != null) {
            RobotPosition.robotPose = follower.getPose();
            RobotPosition.isPoseSet = true;
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
                                shooter.SetTarget(Shooter.AUDIENCE_RPM,Shooter.AUDIENCE_RPM)
                        ),
                        transfer.SetAutomaticTransfer(true),
                        new ShootArtifacts(shooter, spindexer, transfer, intake,touch),
                        transfer.SetAutomaticTransfer(false),
                        // Turn off the motors and servos
                        shooter.SetTarget(0, 0),
                        intake.Stop(),
                        transfer.TransferStop(),
                        transfer.IntakeDoorStop(),

                        new FollowPathCommand(follower, paths.toSpikeOne).setGlobalMaxPower(0.6),
                        intake.In(),
                        spindexer.DirectPower(0.3),
                        transfer.IntakeDoorOut(),


                        new FollowPathCommand(follower, paths.collectSpikeOne).setGlobalMaxPower(1),





                        shooter.SetTarget(Shooter.AUDIENCE_RPM, Shooter.AUDIENCE_RPM),
                        new ParallelCommandGroup(
                        new FollowPathCommand(follower, paths.toShootSpikeOne),
                                new SequentialCommandGroup(
                                        spindexer.DirectPower(0.3),
                                        intake.In(),
                                        new WaitCommand(500),
                                        spindexer.DirectPower(0),
                                        intake.Stop()
                                )
                                ),
                        transfer.IntakeDoorOut(),

                        transfer.SetAutomaticTransfer(true),
                        new ShootArtifacts(shooter, spindexer, transfer, intake,touch),
                        transfer.SetAutomaticTransfer(false),
                        // Turn off the motors and servos

                        //new ParallelRaceGroup(
                            new RepeatCommand(
                                    new SequentialCommandGroup(

                                            shooter.SetTarget(0, 0),
                                            intake.Stop(),
                                            transfer.TransferStop(),
                                            transfer.IntakeDoorStop(),

                                            new FollowPathCommand(follower, paths.toHumanPlayer).setGlobalMaxPower(0.6),

                                            transfer.TransferIn(),
                                            new ParallelCommandGroup(
                                                 spindexer.DirectPower(0.3),
                                                 transfer.IntakeDoorOut(),
                                                    intake.In()
                                                 ),

                                            new FollowPathCommand(follower, paths.collectHumanPlayer).setGlobalMaxPower(1),


                                            shooter.SetTarget(Shooter.AUDIENCE_RPM, Shooter.AUDIENCE_RPM),
                                            transfer.TransferStop(),

                                            new ParallelCommandGroup(
                                                    new FollowPathCommand(follower, paths.toShootHumanPlayer),
                                                    new SequentialCommandGroup(
                                                            spindexer.DirectPower(0.3),
                                                            intake.In(),
                                                            new WaitCommand(500),
                                                            spindexer.DirectPower(0),
                                                            intake.Stop()
                                                    )
                                            ),
                                            transfer.IntakeDoorOut(),
                                            transfer.SetAutomaticTransfer(true),
                                            new ShootArtifacts(shooter, spindexer, transfer, intake,touch),
                                            transfer.SetAutomaticTransfer(false),
                                            // Turn off the motors and servos
                                            shooter.SetTarget(0, 0),
                                            intake.Stop(),
                                            transfer.TransferStop(),
                                            transfer.IntakeDoorStop()
                                    )
                                   ),
                                 // new WaitUntilCommand(()-> timer.milliseconds() >=  28000)
                                //),
                        // Turn off the motors and servos

                        shooter.SetTarget(0, 0),
                        intake.Stop(),
                        transfer.TransferStop(),
                        transfer.IntakeDoorStop(),
                        new FollowPathCommand(follower,paths.toParkSpikeTwo)
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

        telemetry.addData("loop end time", timer.milliseconds());

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

        public PathChain toHumanPlayer;
        public PathChain collectHumanPlayer;
        public PathChain toShootHumanPlayer;

        public PathChain toSpikeOne;
        public PathChain collectSpikeOne;
        public PathChain toShootSpikeOne;
        public PathChain toSpikeTwo;
        public PathChain collectSpikeTwo;
        public PathChain toParkSpikeTwo;
        public PathChain toShootSpikeTwo;
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

                toHumanPlayer = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(59.440, 17.328), new Pose(46, 20.328))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(294.935), Math.toRadians(180))
                        .build();

                collectHumanPlayer = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(46.000, 17.328), new Pose(24.000, 20.328))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))

                        .addPath(
                                new BezierLine(new Pose(24, 17.328), new Pose(17.000, 20.328))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .addPath(
                                new BezierLine(new Pose(17, 17.328), new Pose(24, 20.328))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))

                        .addPath(
                                new BezierLine(new Pose(24, 17.328), new Pose(17.000, 20.328))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .addPath(
                                new BezierLine(new Pose(17, 17.328), new Pose(24, 20.328))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build();



                toShootHumanPlayer = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Pose(24.000, 20.328),
                                        new Pose(59.440, 17.328)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(294.935))
                        .build();

                toSpikeOne = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(59.440, 17.328), new Pose(46.000, 40.50))
                        )
                        .setLinearHeadingInterpolation(
                                Math.toRadians(294.935),
                                Math.toRadians(180)
                        )
                        .build();

                collectSpikeOne = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(46.000, 40.50), new Pose(24.000, 40.50))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))

                        .addPath(
                                new BezierLine(new Pose(24, 40.50), new Pose(17.000, 40.50))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .addPath(
                                new BezierLine(new Pose(17, 40.50), new Pose(24, 40.50))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build();

                toShootSpikeOne = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(24.000, 40.50), new Pose(59.440, 17.328))
                        )
                        .setLinearHeadingInterpolation(
                                Math.toRadians(180),
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
                                new BezierLine(new Pose(46.000, 65.500), new Pose(24.000, 65.500))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))

                        .addPath(
                                new BezierLine(new Pose(24, 65.500), new Pose(17.000, 65.500))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .addPath(
                                new BezierLine(new Pose(17, 65.500), new Pose(24, 65.500))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build();

                toShootSpikeTwo = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(24.000, 65.500), new Pose(59.440, 17.328))
                        )
                        .setLinearHeadingInterpolation(
                                Math.toRadians(180),
                                Math.toRadians(294.935)
                        )
                        .build();

                toParkSpikeTwo = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(59.440, 17.328), new Pose(40, 34))
                        )
                        .setLinearHeadingInterpolation(
                                Math.toRadians(294.935),
                                Math.toRadians(180)
                        )
                        .build();
            } else if (team.equals(Team.RED)) {
                shootPreload = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(81.000, 9.000),

                                        new Pose(83.000, 17.328)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-114.14))

                        .build();

                toHumanPlayer = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(83.000, 17.328), new Pose(109, 20.328))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-114.14), Math.toRadians(0))
                        .build();

                collectHumanPlayer = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(109.000, 17.328), new Pose(132.000, 20.328))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0))

                        .addPath(
                                new BezierLine(new Pose(132, 17.328), new Pose(136.000, 20.328))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .addPath(
                                new BezierLine(new Pose(136.000, 17.328), new Pose(132, 20.328))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0))

                        .addPath(
                                new BezierLine(new Pose(132, 17.328), new Pose(136.000, 20.328))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .addPath(
                                new BezierLine(new Pose(136.000, 17.328), new Pose(132, 20.328))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .build();



                toShootHumanPlayer = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Pose(132.000, 20.328),
                                        new Pose(59.440, 17.328)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-114.14))
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


            }
        }

//		private Pose pose(double x, double y) {
//			if (team == Team.RED) {
//				return new Pose(144 - x, y);
//			}
//			return new Pose(x, y);
//		}
//
//		private double heading(double degrees) {
//			double radians = Math.toRadians(degrees);
//			if (team == Team.RED) {
//				return Math.PI - radians;
//			}
//			return radians;
//		}
    }
}