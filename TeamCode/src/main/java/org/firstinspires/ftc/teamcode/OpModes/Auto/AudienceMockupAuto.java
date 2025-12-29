package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;


@Autonomous
public class AudienceMockupAuto extends OpMode {

    MecanumDrive drive;
    Pose2d startingPose = new Pose2d(60,-9,Math.toRadians(0));
    public static double shootingX = 57, shootingY = -23; // this is the position used shooting blank right now
    public static double  Goalx = -72, Goaly = -72; // this is the position of the goal blank right now

    public static double AngleofShot(double x, double y){
        double diffx = Goalx - x;
        double diffy = Goaly - y;

        double angle = Math.atan2(-diffy, -diffx);
        return angle;
    }
    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap,startingPose);
    }

    @Override
    public void start(){
        Actions.runBlocking(
                drive.actionBuilder(startingPose)


                        .strafeToLinearHeading(new Vector2d(55, -12), Math.toRadians(23))
                        .waitSeconds(2.8)


                        .setTangent(Math.toRadians(-90))
                        .splineToLinearHeading(new Pose2d(27,-24,Math.toRadians(270)),Math.toRadians(270))
                        .lineToY(-46)
                        .setTangent(Math.toRadians(-90))
                        .splineToLinearHeading(new Pose2d(55,-12,Math.toRadians(23)),Math.toRadians(23))
                        .waitSeconds(2.8)

                        .setTangent(Math.toRadians(-90))
                        .splineToLinearHeading(new Pose2d(4,-24,Math.toRadians(270)),Math.toRadians(270))
                        .lineToY(-46)
                        .setTangent(Math.toRadians(-90))
                        .splineToLinearHeading(new Pose2d(55,-12,Math.toRadians(23)),Math.toRadians(23))
                        .waitSeconds(2.8)

                        .setTangent(Math.toRadians(-90))
                        .splineToLinearHeading(new Pose2d(-20,-24,Math.toRadians(270)),Math.toRadians(270))
                        .lineToY(-37)
                        .setTangent(Math.toRadians(-90))
                        .splineToLinearHeading(new Pose2d(55,-12,Math.toRadians(23)),Math.toRadians(23))

//                        // PRE LOAD
//
//                        // most likey we would start the camera init now
//                        .strafeToLinearHeading(new Vector2d(shootingX, shootingY), Math.toRadians(23)) // move to shoot
//                        // checks the apriltag to find corect order
//                        .waitSeconds(2) // fires the artifact in correct order
//
//                        // SPIKE MARK ONE
//
//                        .strafeToLinearHeading(new Vector2d(27,-24), Math.toRadians(270)) // turns to the first spike zone
//                        // the brush would also start running
//                        .waitSeconds(0.03)
//                        .strafeTo(new Vector2d(27,-57)) // pick up the first spike zone
//                        .strafeToLinearHeading(new Vector2d(shootingX, shootingY), Math.toRadians(23)) // goes to
//                        // shoot the brand new loaded
//                        .waitSeconds(2) // fires the artifact in correct order
//
//                        // SPIKE MARK TWO
//
//                        .strafeToLinearHeading(new Vector2d(4,-24), Math.toRadians(270))
//                        .waitSeconds(0.03)
//                        .strafeTo(new Vector2d(4,-57)) // pick up the Second spike zone
//                        .strafeToLinearHeading(new Vector2d(shootingX, shootingY), Math.toRadians(23))// goes to
//                        // shoot the brand new loaded
//                        .waitSeconds(2) // fires the artifact in corect order
//
//
//                        // SPIKE MARK THREE
//
//                        .strafeToLinearHeading(new Vector2d(-20,-24), Math.toRadians(270))
//                        .waitSeconds(0.03)
//                        .strafeTo(new Vector2d(-20,-47)) // pick up the three spike zone
//                        .strafeToLinearHeading(new Vector2d(shootingX, shootingY), Math.toRadians(23))
//                        .waitSeconds(2) // fires the artifact in corect order
                        .build()

        );
    }

    @Override
    public void loop() {

        drive.updatePoseEstimate();

    }
}
