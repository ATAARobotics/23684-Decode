package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.teamcode.LifecycleManagementUtilities.SubsystemUpdater;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.ShootPatterns;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Utilities.ActionScheduler;


@Autonomous
public class ShootPaternTest extends OpMode {
    Shooter shooter;
    Transfer transfer;
    Intake intake;
    Spindexer spindexer;
    ShootPatterns shootPatterns;

    MecanumDrive drive;

    DcMotor intakeMotor;

    Pose2d startPose = new Pose2d(35, -23, Math.toRadians(-90));
    private TrajectoryActionBuilder trajectoryToPickUpOne;
    private TrajectoryActionBuilder trajectoryToBackOut;


     ActionScheduler actionScheduler;
    @Override
    public void init() {
        Shooter.initialize(hardwareMap);
        shooter = Shooter.getInstance();

        Transfer.initialize(hardwareMap);
        transfer = Transfer.getInstance();

        Intake.initialize(hardwareMap);
        intake = Intake.getInstance();

        Spindexer.initialize(hardwareMap);
        spindexer = Spindexer.getInstance();

        shootPatterns = new ShootPatterns(hardwareMap);
        actionScheduler = ActionScheduler.getInstance();

        drive = new MecanumDrive(hardwareMap,startPose);

        trajectoryToPickUpOne = drive.actionBuilder(new Pose2d(35, -23, Math.toRadians(-90)))
                .strafeTo(new Vector2d(35,-68.5));


        trajectoryToBackOut = drive.actionBuilder(new Pose2d(35, -68.5, Math.toRadians(-90)))
                .strafeTo(new Vector2d(35,-23));




    }

    @Override
    public void start() {

        Actions.runBlocking(
                new SequentialAction(
                        spindexer.setDirectPower(1),
                        new ParallelAction(
                                intake.in(),
                                transfer.intakeDoorForward()
                        ),
                        new InstantAction(() ->
                                drive.PARAMS.maxWheelVel = 10
                        ),

                         trajectoryToPickUpOne.build(),
                        new InstantAction(() ->
                                drive.PARAMS.maxWheelVel = 50
                        ),
                        trajectoryToBackOut.build(),
                        new InstantAction(()->{
                            telemetry.addLine("Patrick, we did it!");
                        })


                )

        );

    }

    @Override
    public void loop() {
        actionScheduler.update();


    }
}
