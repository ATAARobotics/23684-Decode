package org.firstinspires.ftc.teamcode.Subsystem;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.LowPassFilter;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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

import java.util.List;


public class Limelight {



    private Limelight3A limelight;


    Team team;
    double heading;


    private IMU imu;

    public double normalizeAngle(double angle) {
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        while (angle > Math.PI) angle -= 2 * Math.PI;
        return angle;

    }

    LowPassFilter xLowPass;

    LowPassFilter yLowPass;

    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);




        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        );
        imu.initialize(new IMU.Parameters(orientation));
        imu.resetYaw();

        team = Team.BLUE;

        xLowPass = new LowPassFilter(0.8);
        yLowPass = new LowPassFilter(0.8);
    }

    public void StartHeading(double heading){

    }

    public void start() {
        limelight.start();
    }
    public void updateLoPass(double followheading){
        xLowPass.update(PPVisionPoseRaw().getX(),1);
        yLowPass.update(PPVisionPoseRaw().getY(),1);

        heading = followheading;

    }



    public void Telemetry(Telemetry telemetry) {


        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

        //limelight.updateRobotOrientation(heading - 270);
        LLResult llResult = limelight.getLatestResult();

		telemetry.addLine(new Pose(xLowPass.getState(),yLowPass.getState(),heading).toString());

        telemetry.addData("heading", heading);


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

       // limelight.updateRobotOrientation(heading);
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

	private Pose PPVisionPoseRaw() {
        double theta = -Math.PI / 2;
        double newHeading = MathFunctions.normalizeAngle(heading + theta);
        limelight.updateRobotOrientation(Math.toDegrees(newHeading));
		LLResult llResult = limelight.getLatestResult();

		if (llResult != null && llResult.isValid()) {
			Pose3D botPose = llResult.getBotpose_MT2();

            Pose2D ftcPose = new Pose2D(DistanceUnit.METER, botPose.getPosition().x ,botPose.getPosition().y, AngleUnit.RADIANS, Math.toRadians(heading));
			Pose finalPose = new Pose(ftcPose.getY(DistanceUnit.INCH) + 72, 72 - ftcPose.getX(DistanceUnit.INCH));

			return finalPose;
		}

		return new Pose(0, 0, 0);

	}

	public Pose PPVisionPose() {

		LLResult llResult = limelight.getLatestResult();

		if (llResult != null && llResult.isValid()) {

				return new Pose(
                        1.13412* xLowPass.getState() + 61.17806,
                        0.949838* yLowPass.getState() + 64.9307
				);


		}

		return new Pose(0, 0, 0);

	}

//	public double XCorrection(double xpose){
//		return 1.2125*xpose + 76.32341;
//	}
//
//	public double YCorrection(double ypose){
//		return 0.838485*ypose + 65.20144;
//	}

	public double HCorrection(double angle){
		return angle;
	}

	public double DistanceFromGoal(Team team) {

		LLResult llResult = limelight.getLatestResult();

		if (llResult != null && llResult.isValid()) {
			Pose3D botPose = llResult.getBotpose();

			List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
			for (LLResultTypes.FiducialResult fiducial : fiducialResults) {

				if (team == Team.BLUE) {
					if (fiducial.getFiducialId() == 20) {

						int id = fiducial.getFiducialId();
						VectorF target = AprilTagGameDatabase.getDecodeTagLibrary().lookupTag(id).fieldPosition.multiplied(0.0254f);
						VectorF robotPose = new VectorF((float) botPose.getPosition().x, (float) botPose.getPosition().y, 0.7493f);
						VectorF targetDis = target.subtracted(robotPose);
						return targetDis.magnitude();
					}
				} else if (team == Team.RED) {
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
				if (fiducial.getFiducialId() == 20 || fiducial.getFiducialId() == 24) {
					return true;
				}
			}

		}
		return false;
	}
}