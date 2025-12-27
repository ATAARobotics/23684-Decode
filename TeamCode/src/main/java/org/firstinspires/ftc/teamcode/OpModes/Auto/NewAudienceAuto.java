package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.ShootPatterns;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Utilities.ActionScheduler;

@Autonomous
public class NewAudienceAuto extends OpMode {

    MecanumDrive drive;
    Transfer transfer;
    Spindexer spindexer;

    ShootPatterns shootPatterns;

    Intake intake;
    Shooter shooter;

    ActionScheduler actionScheduler;
    Pose2d startingPose = new Pose2d(63,-9,Math.toRadians(0));
    public static double shootingX = 57, shootingY = -23; // this is the position used
    public static double  Goalx = -72, Goaly = -72; // this is the position of the goal not used right now but hes still a part of the team

    public TrajectoryActionBuilder toShootPreloaded;
    public  TrajectoryActionBuilder toShootone;
    public TrajectoryActionBuilder toPickupOne;
    public TrajectoryActionBuilder toColectOne;

    public  TrajectoryActionBuilder toShootTwo;
    public TrajectoryActionBuilder toPickupTwo;
    public TrajectoryActionBuilder toColectTwo;

    public  TrajectoryActionBuilder toShootThree;
    public TrajectoryActionBuilder toPickupThree;
    public TrajectoryActionBuilder toColectThree;


    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap,startingPose);

        toShootPreloaded = drive.actionBuilder(startingPose)
                .strafeToLinearHeading(new Vector2d(shootingX, shootingY), Math.toRadians(23));

        // one
        toColectOne = drive.actionBuilder(new Pose2d(shootingX,shootingY,Math.toRadians(23)))
                .strafeToLinearHeading(new Vector2d(33,-24), Math.toRadians(270));

        toPickupOne = drive.actionBuilder(new Pose2d(33,-24,Math.toRadians(270)))
                .waitSeconds(0.03)
                .strafeToLinearHeading(new Vector2d(33,-50),Math.toRadians(270));

        toShootone = drive.actionBuilder(new Pose2d(33,-50,Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(shootingX, shootingY), Math.toRadians(23));

        // two
        toColectTwo = drive.actionBuilder(new Pose2d(shootingX,shootingY,Math.toRadians(23)))
                .strafeToLinearHeading(new Vector2d(4,-24), Math.toRadians(270));

        toPickupTwo = drive.actionBuilder(new Pose2d(4,-24,Math.toRadians(270)))
                .waitSeconds(0.03)
                .strafeToLinearHeading(new Vector2d(4,-50),Math.toRadians(270));

        toShootTwo = drive.actionBuilder(new Pose2d(4,-50,Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(shootingX, shootingY), Math.toRadians(23));

        //three
        toColectThree = drive.actionBuilder(new Pose2d(shootingX,shootingY,Math.toRadians(23)))
                .strafeToLinearHeading(new Vector2d(-20,-24), Math.toRadians(270));

        toPickupThree = drive.actionBuilder(new Pose2d(-20,-24,Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(-20,-45),Math.toRadians(270));

        toShootThree = drive.actionBuilder(new Pose2d(-20,-45,Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(shootingX, shootingY), Math.toRadians(23));


        shootPatterns = new ShootPatterns(hardwareMap);

        Shooter.initialize(hardwareMap);
        shooter = Shooter.getInstance();

        Transfer.initialize(hardwareMap);
        transfer = Transfer.getInstance();

        Intake.initialize(hardwareMap);
        intake = Intake.getInstance();

        Spindexer.initialize(hardwareMap);
        spindexer = Spindexer.getInstance();

        actionScheduler = ActionScheduler.getInstance();
    }

    @Override
    public void start(){

        actionScheduler.schedule(
                new SequentialAction(
                        new ParallelAction(
                            shooter.run(Shooter.AUDIENCE_RPM, Shooter.AUDIENCE_RPM),
                            transfer.intakeDoorForward(),
                            toShootPreloaded.build()
                        ),
                        // Shoot preloaded
                        shootPatterns.ShootPatern(123),
                        transfer.transferOut(),
                        toColectOne.build(),
                        new ParallelAction(
                                intake.in(),
                                spindexer.setDirectPower(1)
                        ),
                        toPickupOne.build(),
                        new ParallelAction(
                           intake.out(),
                            toShootone.build(),
                            shooter.run(Shooter.AUDIENCE_RPM,Shooter.AUDIENCE_RPM)
                         ),
                        //Shoot spike mark one
                        shootPatterns.ShootPatern(123),
                        transfer.transferOut(),
                        toColectTwo.build(),
                        new ParallelAction(
                                intake.in(),
                                spindexer.setDirectPower(1)
                        ),
                        toPickupTwo.build(),
                        new ParallelAction(
                                intake.out(),
                                toShootTwo.build(),
                                shooter.run(Shooter.AUDIENCE_RPM,Shooter.AUDIENCE_RPM)
                        ),
                        //Shoot spike mark twah
                        shootPatterns.ShootPatern(123),
                        transfer.transferOut(),
                        toColectThree.build(),
                        new ParallelAction(
                                intake.in(),
                                spindexer.setDirectPower(1)
                        ),
                        toPickupThree.build(),
                        new ParallelAction(
                                intake.out(),
                                toShootThree.build(),
                                shooter.run(Shooter.AUDIENCE_RPM,Shooter.AUDIENCE_RPM)
                        ),
                        //Shoot spike mark three
                        shootPatterns.ShootPatern(123)
                )

        );


    }

    @Override
    public void loop() {
        actionScheduler.update();
        drive.updatePoseEstimate();


    }
}
