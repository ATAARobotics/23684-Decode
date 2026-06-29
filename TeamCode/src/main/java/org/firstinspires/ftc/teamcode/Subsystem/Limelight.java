package org.firstinspires.ftc.teamcode.Subsystem;

import com.pedropathing.control.LowPassFilter;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Utils.RobotConfig;
import org.firstinspires.ftc.teamcode.Utils.Team;

import java.util.List;


public class Limelight extends SubsystemBase {
	public static final int BLUEGOAL = 20;
	public static final int REDGOAL = 24;

	private final Limelight3A limelight;
	private final LowPassFilter xLowPassFilter;
	private final LowPassFilter yLowPassFilter;

	private double heading;

	public Limelight(HardwareMap hardwareMap) {
		limelight = hardwareMap.get(Limelight3A.class, "limelight");
		limelight.pipelineSwitch(0);
		limelight.start();

		xLowPassFilter = new LowPassFilter(0.4);
		yLowPassFilter = new LowPassFilter(0.4);
	}

	public Limelight(HardwareMap hardwareMap, Follower follower) {
		this(hardwareMap);
		// Pass-through kept for legacy callers; the follower reference is no
		// longer required for any Limelight subsystem behavior.
	}

	public void start() {
		limelight.start();
	}

	public double calculateShotAngle(double x, double y, double goalX, double goalY) {
		double deltaX = Math.abs(goalX - x);
		double deltaY = Math.abs(goalY - y);
		double tanAngle = Math.atan2(deltaY, deltaX);
		return MathFunctions.normalizeAngle(tanAngle);
	}

	public double calculateShotAngle2(double x, double y, double goalX, double goalY) {
		double deltaX = Math.abs(goalX - x);
		double deltaY = Math.abs(goalY - y);
		double tanAngle = Math.atan2(deltaY, deltaX);
		double insideAngle = Math.toRadians(180) - (Math.toRadians(90) + tanAngle);
		return Math.toRadians(180) - insideAngle;
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
		if (RobotConfig.COMPETITION) return;
		LLResult llResult = limelight.getLatestResult();

		telemetry.addData("Filtered Pose", new Pose(xLowPassFilter.getState(), yLowPassFilter.getState(), heading).toString());

		if (llResult != null && llResult.isValid()) {
			Pose3D botPose = llResult.getBotpose_MT2();
			telemetry.addData("LL Latency", llResult.getCaptureLatency() + llResult.getTargetingLatency());
			telemetry.addData("LL MT2 Pose", botPose.toString());

			telemetry.addData("Blue Goal Found?", goalsFound(Team.BLUE) ? "true" : "false");
			telemetry.addData("Red Goal Found?", goalsFound(Team.RED) ? "true" : "false");

			for (LLResultTypes.FiducialResult fiducial : llResult.getFiducialResults()) {
				telemetry.addLine(fiducial.getFiducialId() + " : " + fiducial.getTargetXDegrees());
			}
		}
	}

	public boolean goalsFound(Team tag) {
		LLResult llResult = limelight.getLatestResult();

		if (llResult != null && llResult.isValid()) {
			List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
			for (LLResultTypes.FiducialResult fiducial : fiducialResults) {
				if (tag == Team.BLUE) {
					if (fiducial.getFiducialId() == BLUEGOAL) {
						return true;
					}
				} else if (tag == Team.RED) {
					if (fiducial.getFiducialId() == BLUEGOAL) {
						return true;
					}
				}

			}
		}
		return false;
	}

	public double AngleFrom(Team tag) {
		LLResult llResult = limelight.getLatestResult();
		if (llResult == null || !llResult.isValid()) return 0;

		for (LLResultTypes.FiducialResult fiducial : llResult.getFiducialResults()) {
			if (tag == Team.BLUE && fiducial.getFiducialId() == BLUEGOAL) {
				return fiducial.getTargetXDegrees();
			}
			if (tag == Team.RED && fiducial.getFiducialId() == REDGOAL) {
				return fiducial.getTargetXDegrees();
			}
		}
		return 0;
	}
}
