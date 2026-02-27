package org.firstinspires.ftc.teamcode.OpModes.Auto.Modular;

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
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
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

import java.util.ArrayList;
import java.util.List;

public abstract class ModularAuto extends OpMode {
    public static int SPIKE_COLLECTION_WAIT = 20;

    protected Follower follower;
    protected CommandScheduler scheduler;
    protected Intake intake;
    protected Shooter shooter;
    protected Spindexer spindexer;
    protected Transfer transfer;
    protected Touch touch;
    protected TelemetryManager panelsTelemetry;

    private List<RouteStep> route = new ArrayList<>();
    private Pose currentExpectedPose;

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
        touch = new Touch(hardwareMap);
        touch.init();

        setRoute();

        panelsTelemetry.debug("Status", "Modular Auto Initialized");
        panelsTelemetry.update(telemetry);
    }

    protected abstract Pose getStartingPose();
    protected abstract Team getTeam();
    protected abstract void setRoute();

    protected void addStep(RouteStep step) {
        route.add(step);
    }

    @Override
    public void start() {
        spindexer.zeroSpindexer();
        
        SequentialCommandGroup fullAuto = new SequentialCommandGroup();
        
        for (RouteStep step : route) {
            fullAuto.addCommands(getCommandForStep(step));
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
                        getShootSequence()
                );

            case COLLECT_SPIKE_1:
                return getCollectSpikeCommand(1);

            case COLLECT_SPIKE_2:
                return getCollectSpikeCommand(2);

            case COLLECT_SPIKE_3:
                return getCollectSpikeCommand(3);

            case COLLECT_HUMAN_PLAYER:
                return getCollectHumanPlayerCommand();

            case SHOOT:
                return getShootStepCommand();

            case PARK:
                Pose parkPose = team == Team.BLUE ? PoseDatabase.BLUE_PARK : PoseDatabase.RED_PARK;
                PathChain parkPath = follower.pathBuilder()
                        .addPath(new BezierLine(currentExpectedPose, parkPose))
                        .setLinearHeadingInterpolation(currentExpectedPose.getHeading(), parkPose.getHeading())
                        .build();
                currentExpectedPose = parkPose;
                return new FollowPathCommand(follower, parkPath);

            default:
                return new WaitCommand(0);
        }
    }

    private Command getCollectSpikeCommand(int spikeNum) {
        Team team = getTeam();
        Pose intermediate, collect;
        if (team == Team.BLUE) {
            if (spikeNum == 1) { intermediate = PoseDatabase.BLUE_SPIKE_1_INTERMEDIATE; collect = PoseDatabase.BLUE_SPIKE_1_COLLECT; }
            else if (spikeNum == 2) { intermediate = PoseDatabase.BLUE_SPIKE_2_INTERMEDIATE; collect = PoseDatabase.BLUE_SPIKE_2_COLLECT; }
            else { intermediate = PoseDatabase.BLUE_SPIKE_3_INTERMEDIATE; collect = PoseDatabase.BLUE_SPIKE_3_COLLECT; }
        } else {
            if (spikeNum == 1) { intermediate = PoseDatabase.RED_SPIKE_1_INTERMEDIATE; collect = PoseDatabase.RED_SPIKE_1_COLLECT; }
            else if (spikeNum == 2) { intermediate = PoseDatabase.RED_SPIKE_2_INTERMEDIATE; collect = PoseDatabase.RED_SPIKE_2_COLLECT; }
            else { intermediate = PoseDatabase.RED_SPIKE_3_INTERMEDIATE; collect = PoseDatabase.RED_SPIKE_3_COLLECT; }
        }

        PathChain toSpike = follower.pathBuilder()
                .addPath(new BezierLine(currentExpectedPose, intermediate))
                .setLinearHeadingInterpolation(currentExpectedPose.getHeading(), intermediate.getHeading())
                .build();

        PathChain collectPath = follower.pathBuilder()
                .addPath(new BezierLine(intermediate, collect))
                .setConstantHeadingInterpolation(collect.getHeading())
                .build();

        currentExpectedPose = collect;

        return new SequentialCommandGroup(
                new FollowPathCommand(follower, toSpike).setGlobalMaxPower(0.6),
                transfer.TransferIn(),
                new ParallelCommandGroup(
                        intake.In(),
                        spindexer.DirectPower(0.3),
                        transfer.IntakeDoorOut()
                ),
                new FollowPathCommand(follower, collectPath).setGlobalMaxPower(1),
                new WaitCommand(SPIKE_COLLECTION_WAIT)
        );
    }

    private Command getCollectHumanPlayerCommand() {
        Team team = getTeam();
        Pose intermediate = (team == Team.BLUE) ? PoseDatabase.BLUE_HUMAN_PLAYER_INTERMEDIATE : PoseDatabase.RED_HUMAN_PLAYER_INTERMEDIATE;
        Pose collect = (team == Team.BLUE) ? PoseDatabase.BLUE_HUMAN_PLAYER_COLLECT : PoseDatabase.RED_HUMAN_PLAYER_COLLECT;
        Pose wiggle = (team == Team.BLUE) ? PoseDatabase.BLUE_HUMAN_PLAYER_COLLECT_WIGGLE : PoseDatabase.RED_HUMAN_PLAYER_COLLECT_WIGGLE;

        PathChain toHP = follower.pathBuilder()
                .addPath(new BezierLine(currentExpectedPose, intermediate))
                .setLinearHeadingInterpolation(currentExpectedPose.getHeading(), intermediate.getHeading())
                .build();

        PathChain collectPath = follower.pathBuilder()
                .addPath(new BezierLine(intermediate, collect))
                .setConstantHeadingInterpolation(collect.getHeading())
                .addPath(new BezierLine(collect, wiggle))
                .setConstantHeadingInterpolation(collect.getHeading())
                .addPath(new BezierLine(wiggle, collect))
                .setConstantHeadingInterpolation(collect.getHeading())
                .build();

        currentExpectedPose = collect;

        return new SequentialCommandGroup(
                new FollowPathCommand(follower, toHP).setGlobalMaxPower(0.6),
                intake.In(),
                spindexer.DirectPower(0.3),
                transfer.IntakeDoorOut(),
                new FollowPathCommand(follower, collectPath).setGlobalMaxPower(1),
                new WaitCommand(SPIKE_COLLECTION_WAIT)
        );
    }

    private Command getShootStepCommand() {
        Team team = getTeam();
        Pose shootPose = PoseDatabase.getShootPose(team);
        PathChain toShoot;

        // Special handling for Spike 3 return path
        if (currentExpectedPose.equals(PoseDatabase.BLUE_SPIKE_3_COLLECT) && team == Team.BLUE) {
             toShoot = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(15.000, 89.500),
                        new Pose(52.449, 88.976),
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

        currentExpectedPose = shootPose;

        return new SequentialCommandGroup(
                intake.Slow(),
                shooter.SetTarget(Shooter.AUDIENCE_RPM, Shooter.AUDIENCE_RPM),
                spindexer.DirectPower(0),
                new FollowPathCommand(follower, toShoot),
                transfer.IntakeDoorOut(),
                getShootSequence()
        );
    }

    private Command getShootSequence() {
        return new SequentialCommandGroup(
                transfer.SetAutomaticTransfer(true),
                new ShootArtifacts(shooter, spindexer, transfer, intake, touch),
                transfer.SetAutomaticTransfer(false),
                shooter.SetTarget(0, 0),
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
        touch.Update(transfer);
        
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
        CommandScheduler.getInstance().reset();
    }
}
