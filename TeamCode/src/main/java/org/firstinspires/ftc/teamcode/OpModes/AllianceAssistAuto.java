package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;
import org.firstinspires.ftc.teamcode.Subsystem.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystem.Transfer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.BooleanSupplier;

@Configurable // Panels
public class AllianceAssistAuto extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    CommandScheduler scheduler;

    Intake intake;
    Spindexer spindexer;
    Transfer transfer;
    Shooter shooter;
    public double waitTimeForOtherTeam = 25;

    private ElapsedTime OpModeTimer = new ElapsedTime();

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

    public void start(){
        OpModeTimer.reset();
        scheduler.schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower,paths.toShootPreload),
                                shooter.ToTarget(Shooter.AUDIENCE_RPM, Shooter.AUDIENCE_RPM),
                                spindexer.DirectPower(1),
                                transfer.IntakeDoorIn(),
                                transfer.TransferIn(),
                                intake.Slow()
                        ),
                        transfer.TransferOut(),
                        shooter.MindlessToTarget(Shooter.AUDIENCE_RPM, Shooter.AUDIENCE_RPM).withTimeout(2800), // shoot preloaded
                        new ParallelCommandGroup(
                             shooter.Stop(),
                             intake.In(),
                             transfer.TransferIn(),
                             new FollowPathCommand(follower,paths.toSpikeOne)
                        ),
                         new WaitCommand(30),
                         new FollowPathCommand(follower,paths.collectSpikeOne),
                         new ParallelCommandGroup(
                                new FollowPathCommand(follower,paths.toShootSpikeOne),
                                intake.Out(),
                                shooter.ToTarget(Shooter.AUDIENCE_RPM, Shooter.AUDIENCE_RPM)
                         ),
                         transfer.TransferOut(),
                         shooter.MindlessToTarget(Shooter.AUDIENCE_RPM, Shooter.AUDIENCE_RPM).withTimeout(2800), // shoot spike One
                        new ParallelCommandGroup(
                                shooter.Stop(),
                                intake.In(),
                                transfer.TransferIn(),
                                new FollowPathCommand(follower,paths.toHumanPlayer)
                        ),
                        new WaitCommand(30),
                        new FollowPathCommand(follower,paths.collectHumanPlayer),
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower,paths.toShootHumanPlayer),
                                intake.Out(),
                                shooter.ToTarget(Shooter.AUDIENCE_RPM, Shooter.AUDIENCE_RPM)
                        ),
                        transfer.TransferOut(),
                        shooter.MindlessToTarget(Shooter.AUDIENCE_RPM, Shooter.AUDIENCE_RPM).withTimeout(2800), // shoot Humman player
                        new WaitUntilCommand(() -> waitTimeForOtherTeam >= OpModeTimer.seconds())





                )
        );
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain toShootPreload;
        public PathChain toSpikeOne;
        public PathChain collectSpikeOne;
        public PathChain toShootSpikeOne;
        public PathChain toHumanPlayer;
        public PathChain collectHumanPlayer;
        public PathChain toShootHumanPlayer;
        public PathChain toTunnelBack;
        public PathChain toTunnelFront;

        public Paths(Follower follower) {
            toShootPreload = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(63.000, 9.000), new Pose(59.440, 17.328))
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(270),
                            Math.toRadians(294.935)
                    )
                    .build();

            toSpikeOne = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(59.440, 17.328), new Pose(39.102, 35.122))
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(294.935),
                            Math.toRadians(180)
                    )
                    .build();

            collectSpikeOne = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(39.102, 35.122), new Pose(8.663, 35.824))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            toShootSpikeOne = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(8.663, 35.824), new Pose(59.440, 17.328))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(294.935))
                    .build();

            toHumanPlayer = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(59.440, 17.328), new Pose(10.068, 31.844))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(294.935), Math.toRadians(270))
                    .build();

            collectHumanPlayer = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(10.068, 31.844), new Pose(10.068, 11.473))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(270))
                    .build();

            toShootHumanPlayer = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(10.068, 11.473),
                                    new Pose(31.844, 13.112),
                                    new Pose(59.440, 17.328)
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(270),
                            Math.toRadians(294.935)
                    )
                    .build();

            toTunnelBack = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(59.440, 17.328),
                                    new Pose(32.078, 20.839),
                                    new Pose(13.112, 16.390)
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(294.935),
                            Math.toRadians(150)
                    )
                    .build();

            toTunnelFront = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(13.112, 16.390), new Pose(13.815, 49.873))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(150))
                    .build();
        }
    }

}