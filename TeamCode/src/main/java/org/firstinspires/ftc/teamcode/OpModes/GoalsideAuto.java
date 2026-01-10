package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
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


@Configurable // Panels
public class GoalsideAuto extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    CommandScheduler scheduler;
    Intake intake;
    Shooter shooter;
    Spindexer spindexer;
    Transfer transfer;


    public static class Paths {

        public PathChain shootPreload;
        public PathChain toSpikeOne;
        public PathChain collectSpikeOne;
        public PathChain toShootSpikeOne;
        public PathChain toSpikeTwo;
        public PathChain collectSpiketwo;
        public PathChain toShootSpikeTwo;
        public PathChain toSpikeThree;
        public PathChain collectSpikeThree;
        public PathChain toShootSpikeThree;

        public PathChain toAmongus;

        public Paths(Follower follower) {
            shootPreload = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(21.073, 123.863), new Pose(48.000, 84.527))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-36), Math.toRadians(307))
                    .build();

            toSpikeOne = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.000, 84.527), new Pose(42.146, 83.590))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(307), Math.toRadians(180))
                    .build();

            collectSpikeOne = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(42.146, 83.590), new Pose(14.751, 83.590))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            toShootSpikeOne = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(14.751, 83.590), new Pose(48.000, 84.059))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(307))
                    .build();

            toSpikeTwo = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.000, 84.059), new Pose(41.678, 59.707))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(307), Math.toRadians(180))
                    .build();

            collectSpiketwo = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(41.678, 59.707), new Pose(8.663, 59.239))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            toShootSpikeTwo = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(8.663, 59.239),
                                    new Pose(53.151, 64.156),
                                    new Pose(45.424, 46.127),
                                    new Pose(49.405, 70.946),
                                    new Pose(48.000, 84.293)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(307))
                    .build();

            toSpikeThree = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.000, 84.293), new Pose(40.507, 35.824))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(307), Math.toRadians(180))
                    .build();

            collectSpikeThree = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(40.507, 35.824), new Pose(8.898, 34.888))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            toShootSpikeThree = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(8.898, 34.888), new Pose(48.000, 84.761))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(307))
                    .build();


        }
    }

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


        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

    }

    @Override
    public void start() {
        scheduler.schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower,paths.shootPreload),
								shooter.SetTarget(Shooter.GOAL_RPM, Shooter.GOAL_RPM),
                                shooter.WaitForTarget(),
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
                                new FollowPathCommand(follower,paths.toSpikeOne)
                        ),
                        new WaitCommand(30),
                        new FollowPathCommand(follower,paths.collectSpikeOne),
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower,paths.toShootSpikeOne),
                                intake.Out(),
                                shooter.SetTarget(Shooter.GOAL_RPM, Shooter.GOAL_RPM),
								shooter.WaitForTarget()
                        ),
                        transfer.TransferOut(),
                        new ParallelCommandGroup(
                                shooter.SetTarget(0, 0),
                                intake.In(),
                                transfer.TransferIn(),
                                new FollowPathCommand(follower,paths.toSpikeTwo)
                        ),
                        new WaitCommand(30),
                        new FollowPathCommand(follower,paths.collectSpiketwo),
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower,paths.toShootSpikeTwo),
                                intake.Out(),
                                shooter.SetTarget(Shooter.GOAL_RPM, Shooter.GOAL_RPM),
								shooter.WaitForTarget()
                        ),
                        transfer.TransferOut(),
                        new ParallelCommandGroup(
                                shooter.SetTarget(0, 0),
                                intake.In(),
                                transfer.TransferIn(),
                                new FollowPathCommand(follower,paths.toSpikeThree)
                        ),
                        new WaitCommand(30),
                        new FollowPathCommand(follower,paths.collectSpikeThree),
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower,paths.toShootSpikeThree),
                                intake.Out(),
                                shooter.SetTarget(Shooter.GOAL_RPM, Shooter.GOAL_RPM),
								shooter.WaitForTarget()
                        ),
                        transfer.TransferOut()
                )
		);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathin
        scheduler.run();
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }
}
