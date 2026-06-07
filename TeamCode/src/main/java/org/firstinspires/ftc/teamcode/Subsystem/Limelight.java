package org.firstinspires.ftc.teamcode.Subsystem;

import com.pedropathing.control.LowPassFilter;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;


public class Limelight {
	private final Limelight3A limelight;
	double heading;

	public static int BLUEGOAL = 20;
	public static int REDGOAL = 24;

	LowPassFilter xLowPassFilter;
	LowPassFilter yLowPassFilter;

	public Limelight(HardwareMap hardwareMap) {
		limelight = hardwareMap.get(Limelight3A.class, "limelight");
		limelight.pipelineSwitch(0);
		limelight.start();

		xLowPassFilter = new LowPassFilter(0.4);
		yLowPassFilter = new LowPassFilter(0.4);
	}


	public void start() {
		limelight.start();
	}

	/**
	 * Updates the Limelight with the robot's current heading and filters the result.
	 *
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

			telemetry.addData("Blue Goal Found?", blueGoalFound()? "give that man a TRUE" : "false");
			telemetry.addData("Red Goal Found?", redGoalFound()? "give that man a TRUE" : "false");

			List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
			for (LLResultTypes.FiducialResult fiducial : fiducialResults) {

				telemetry.addLine(fiducial.getFiducialId() + " : " + fiducial.getTargetXDegrees());


			}
		}
	}

	public boolean goalsFound() {
		LLResult llResult = limelight.getLatestResult();

		if (llResult != null && llResult.isValid()) {
			List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
			for (LLResultTypes.FiducialResult fiducial : fiducialResults) {
				if (fiducial.getFiducialId() == BLUEGOAL || fiducial.getFiducialId() == REDGOAL) {
					return true;
				}
			}
		}

		return false;
	}

	public boolean redGoalFound() {
		LLResult llResult = limelight.getLatestResult();

		if (llResult != null && llResult.isValid()) {
			List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
			for (LLResultTypes.FiducialResult fiducial : fiducialResults) {
				if (fiducial.getFiducialId() == REDGOAL) {
					return true;
				}
			}
		}

		return false;
	}

	public boolean blueGoalFound() {
		LLResult llResult = limelight.getLatestResult();

		if (llResult != null && llResult.isValid()) {
			List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
			for (LLResultTypes.FiducialResult fiducial : fiducialResults) {
				if (fiducial.getFiducialId() == BLUEGOAL) {
					return true;
				}
			}
		}

		return false;
	}

	public double AngleFrom(int tag) {
		LLResult llResult = limelight.getLatestResult();

		if (llResult != null && llResult.isValid()) {
			List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
			for (LLResultTypes.FiducialResult fiducial : fiducialResults) {
				if (tag == BLUEGOAL) {
					if (fiducial.getFiducialId() == BLUEGOAL) {
						return fiducial.getTargetXDegrees();
					}
				} else if (tag == REDGOAL) {
					if (fiducial.getFiducialId() == BLUEGOAL) {
						return fiducial.getTargetXDegrees();
					}
				}

			}
		}
		return 0;

	}
}