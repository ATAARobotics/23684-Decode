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
	double heading;
	LowPassFilter xLowPassFilter;
	LowPassFilter yLowPassFilter;
	private final Limelight3A limelight;

	public Limelight(HardwareMap hardwareMap) {
		limelight = hardwareMap.get(Limelight3A.class, "limelight");
		limelight.pipelineSwitch(0);
		limelight.start();

		xLowPassFilter = new LowPassFilter(0.4);
		yLowPassFilter = new LowPassFilter(0.4);
	}


	public void start(){
		limelight.start();
	}

	/**
	 * Updates the Limelight with the robot's current heading and filters the result.
	 * @param headingRadians The current robot heading in RADIANS (standard PedroPathing output)
	 */
	public void updateLowPass(double headingRadians) {
		this.heading = headingRadians;

		double theta = -Math.PI / 2;
		double headingForLimelight = MathFunctions.normalizeAngle(heading + theta);
		limelight.updateRobotOrientation(Math.toDegrees(headingForLimelight));

		Pose rawPose = PPVisionPoseRaw();
		xLowPassFilter.update(rawPose.getX(), 1);
		yLowPassFilter.update(rawPose.getY(), 1);
	}

	public Pose PPVisionPoseRaw() {
		LLResult llResult = limelight.getLatestResult();

		if (llResult != null && llResult.isValid()) {
			Pose3D botPose = llResult.getBotpose_MT2();

			Pose2D ftcPose = new Pose2D(
					DistanceUnit.METER,
					botPose.getPosition().x,
					botPose.getPosition().y,
					AngleUnit.RADIANS,
					heading
			);

			return new Pose(ftcPose.getY(DistanceUnit.INCH) + 72, 72 - ftcPose.getX(DistanceUnit.INCH));
		}

		return new Pose(0, 0, 0);
	}

	public void Telemetry(Telemetry telemetry) {
		LLResult llResult = limelight.getLatestResult();

		telemetry.addData("Filtered Pose", new Pose(xLowPassFilter.getState(), yLowPassFilter.getState(), heading).toString());

		if (llResult != null && llResult.isValid()) {
			Pose3D botPose = llResult.getBotpose_MT2();
			telemetry.addData("LL Latency", llResult.getCaptureLatency() + llResult.getTargetingLatency());
			telemetry.addData("LL MT2 Pose", botPose.toString());
		}
		telemetry.update();
	}

	public boolean goalsFound() {
		LLResult llResult = limelight.getLatestResult();

		if (llResult != null && llResult.isValid()) {
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