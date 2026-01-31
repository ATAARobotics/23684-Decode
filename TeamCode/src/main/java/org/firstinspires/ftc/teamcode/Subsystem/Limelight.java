package org.firstinspires.ftc.teamcode.Subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Utils.Team;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

import java.util.List;
import java.util.Vector;


public class Limelight {

    private Limelight3A limelight;

    private GoBildaPinpointDriver pinpoint;


    Team team;
    double heading;


    private IMU imu;

    public double normalizeAngle(double angle) {
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        while (angle > Math.PI) angle -= 2 * Math.PI;
        return angle;

    }

    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        );
        imu.initialize(new IMU.Parameters(orientation));
        imu.resetYaw();

        team = Team.BLUE;
    }

    public void StartHeading(double heading){
        pinpoint.setHeading(heading,AngleUnit.DEGREES);
    }

    public void start() {
        limelight.start();
        setHeading(pinpoint.getHeading(AngleUnit.DEGREES));
    }
    public void updateIMU(){
        pinpoint.update();
    }

    public void setHeading(double heading) {
        heading = heading;
    }

    public void Update() {
        limelight.getLatestResult();

    }


    public void Telemetry(Telemetry telemetry) {


        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

        limelight.updateRobotOrientation(heading);
        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose_MT2();
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
            telemetry.addData("BotPose", botPose.toString());
            telemetry.addData("Orientation", botPose.getOrientation().toString());
            telemetry.addData("imuing", imu.getRobotYawPitchRollAngles());


            List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducialResults) {
                int id = fiducial.getFiducialId();
                double distance = fiducial.getRobotPoseTargetSpace().getPosition().y;
                double x = fiducial.getRobotPoseTargetSpace().getPosition().x;
                double z = fiducial.getRobotPoseTargetSpace().getPosition().z;
                VectorF target = AprilTagGameDatabase.getDecodeTagLibrary().lookupTag(id).fieldPosition.multiplied(0.0254f);
                VectorF robotPose = new VectorF((float) botPose.getPosition().x, (float) botPose.getPosition().y, 0.7493f);
                VectorF targetDis = target.subtracted(robotPose);
                telemetry.addLine("Id:" + id + "distance" + targetDis.magnitude());
                //telemetry.addLine("ID: " + id + " x " + x + " y: " + distance + " z: " + z);
            }


        }

        telemetry.update();
    }

    public void Telemetry(TelemetryManager telemetry) {


        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

        limelight.updateRobotOrientation(heading);
        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose();
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
            telemetry.addData("BotPose", botPose.toString());
            telemetry.addData("Orientation", botPose.getOrientation().toString());
            telemetry.addData("imuing", imu.getRobotYawPitchRollAngles());


            List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducialResults) {
                int id = fiducial.getFiducialId();
                double distance = fiducial.getRobotPoseTargetSpace().getPosition().y;
                double x = fiducial.getRobotPoseTargetSpace().getPosition().x;
                double z = fiducial.getRobotPoseTargetSpace().getPosition().z;
                VectorF target = AprilTagGameDatabase.getDecodeTagLibrary().lookupTag(id).fieldPosition.multiplied(0.0254f);
                VectorF robotPose = new VectorF((float) botPose.getPosition().x, (float) botPose.getPosition().y, 0.7493f);
                VectorF targetDis = target.subtracted(robotPose);
                telemetry.addLine("Id:" + id + "distance" + targetDis.magnitude());
                //telemetry.addLine("ID: " + id + " x " + x + " y: " + distance + " z: " + z);
            }


        }

        telemetry.update();
    }

    public Pose PPVisionPose() {

        LLResult llResult = limelight.getLatestResult();
        limelight.updateRobotOrientation(heading);

        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose_MT2();

            Pose finalPose = PoseConverter.pose2DToPose(new Pose2D(DistanceUnit.INCH , botPose.getPosition().x * 39.3701, botPose.getPosition().y * 39.3701, AngleUnit.RADIANS, botPose.getOrientation().getYaw(AngleUnit.RADIANS)), FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);

            return finalPose;
        }

        return new Pose(0, 0, 0);

    }

    public double DistanceFromGoal(Team team) {

        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose();

            List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducialResults) {

                if (team == team.BLUE) {
                    if (fiducial.getFiducialId() == 20) {

                        int id = fiducial.getFiducialId();
                        VectorF target = AprilTagGameDatabase.getDecodeTagLibrary().lookupTag(id).fieldPosition.multiplied(0.0254f);
                        VectorF robotPose = new VectorF((float) botPose.getPosition().x, (float) botPose.getPosition().y, 0.7493f);
                        VectorF targetDis = target.subtracted(robotPose);
                        return targetDis.magnitude();
                    }
                } else if (team == team.RED) {
                    if (fiducial.getFiducialId() == 24) {
                        int id = fiducial.getFiducialId();
                        VectorF target = AprilTagGameDatabase.getDecodeTagLibrary().lookupTag(id).fieldPosition.multiplied(0.0254f);
                        VectorF robotPose = new VectorF((float) botPose.getPosition().x, (float) botPose.getPosition().y, 0.7493f);
                        VectorF targetDis = target.subtracted(robotPose);
                        return targetDis.magnitude();
                    }
                } else {
                    return 0;
                }

            }


        }

        return 0;
    }


    public boolean goalsFound() {

        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose();

            List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducialResults) {
                if (fiducial.getFiducialId() == 20 || fiducial.getFiducialId() == 24){
                    return true;
                }
            }

        }
        return  false;
    }
}