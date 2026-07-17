package org.firstinspires.ftc.teamcode.Subsystem;

import com.pedropathing.control.LowPassFilter;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Utils.Team;
import org.firstinspires.ftc.teamcode.Utils.TeleOpDrive;

import java.util.Collections;
import java.util.List;
import java.util.Set;


public class Limelight extends SubsystemBase {
	private final Limelight3A limelight;
	double heading;

	boolean headingLock = true;
	double currentHeading;
	double targetHeading = Math.toRadians(180);
	double headingCorrection;

	PIDFController headingPIDController;
	TeleOpDrive teleOpDrive;

	public static double P = 0.03, I, D, F = 0.001;

	Follower follower;

	public static int BLUEGOAL = 20;
	public static int REDGOAL = 24;

	LowPassFilter xLowPassFilter;
	LowPassFilter yLowPassFilter;

	public double calculateShotAngle(double x, double y, double goalX, double goalY) {
		double deltaX = goalX - x;
		double deltaY = goalY - y;

		double Tanang = Math.atan2(deltaY,deltaX);
		double finalang = Tanang + Math.PI;
		return MathFunctions.normalizeAngleSigned(finalang);
		// this works! for now... theres a 98.42% it will break k-days
	}
	public Limelight(HardwareMap hardwareMap) {
		limelight = hardwareMap.get(Limelight3A.class, "limelight");
		limelight.pipelineSwitch(0);
		limelight.start();

		xLowPassFilter = new LowPassFilter(0.4);
		yLowPassFilter = new LowPassFilter(0.4);
	}

	public Limelight(HardwareMap hardwareMap,Follower follower) {
		limelight = hardwareMap.get(Limelight3A.class, "limelight");
		limelight.pipelineSwitch(0);
		limelight.start();

		xLowPassFilter = new LowPassFilter(0.4);
		yLowPassFilter = new LowPassFilter(0.4);

		this.follower = follower;
		this.follower.setStartingPose(new Pose(63.450, 9, Math.toRadians(270)));

		headingPIDController = new PIDFController(new PIDFCoefficients(P, I, D, F));

		teleOpDrive = new TeleOpDrive(hardwareMap);
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

			telemetry.addData("Blue Goal Found?", blueGoalFound() ? "give that man a TRUE" : "false");
			telemetry.addData("Distance From Blue Goal", distanceFrom(Team.BLUE));
			telemetry.addData("Red Goal Found?", redGoalFound() ? "give that man a TRUE" : "false");
			telemetry.addData("Distance From Red Goal", distanceFrom(Team.RED));

			List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
			for (LLResultTypes.FiducialResult fiducial : fiducialResults) {

				telemetry.addLine(fiducial.getFiducialId() + " : " + fiducial.getTargetXDegrees());
				telemetry.addLine(fiducial.getFiducialId() + " : " + fiducial.getTargetPoseCameraSpace());


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
					if (fiducial.getFiducialId() == REDGOAL) {
						return true;
					}
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

	public double AngleFrom(Team tag) {
		LLResult llResult = limelight.getLatestResult();

		if (llResult != null && llResult.isValid()) {
			List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
			for (LLResultTypes.FiducialResult fiducial : fiducialResults) {
				if (tag == Team.BLUE) {
					if (fiducial.getFiducialId() == BLUEGOAL) {
						return fiducial.getTargetXDegrees();
					}
				} else if (tag == Team.RED) {
					if (fiducial.getFiducialId() == REDGOAL) {
						return fiducial.getTargetXDegrees();
					}
				}

			}
		}
		return 0;

	}

	public double distanceFrom(Team tag){
		VectorF distance;
		LLResult llResult = limelight.getLatestResult();

		if (llResult != null && llResult.isValid()) {
			List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
			for (LLResultTypes.FiducialResult fiducial : fiducialResults) {
				if (tag == Team.BLUE) {
					if (fiducial.getFiducialId() == BLUEGOAL) {
						float x = (float) fiducial.getTargetPoseRobotSpace().getPosition().x;
						float y = (float) fiducial.getTargetPoseRobotSpace().getPosition().y;
						double z = (float) 0; //fiducial.getTargetPoseRobotSpace().getPosition().z;
						distance = new VectorF(x,y);
						return distance.magnitude();
					}
				} else if (tag == Team.RED) {
					if (fiducial.getFiducialId() == REDGOAL) {
						float x = (float) fiducial.getTargetPoseRobotSpace().getPosition().x;
						float y = (float) fiducial.getTargetPoseRobotSpace().getPosition().y;
						double z = (float) 0; //fiducial.getTargetPoseRobotSpace().getPosition().z;
						distance = new VectorF(x,y);
						return distance.magnitude();
					}
				}

			}
		}
		return 0;
	}

	public Command ShootToGoal(Team tag){
		return new CommandBase() {
			double headingError;
			@Override
			public void initialize() {
				if (tag == Team.BLUE) {
					if (blueGoalFound()) {
						currentHeading = AngleFrom(tag);
						targetHeading = 0;
						headingPIDController.setCoefficients(new PIDFCoefficients(P, I, D, F));

					} else {
						currentHeading = follower.getPose().getHeading();
						targetHeading = calculateShotAngle(follower.getPose().getX(), follower.getPose().getY(), 0, 144);
						headingPIDController.setCoefficients(Constants.followerConstants.coefficientsHeadingPIDF);
					}
				} else if (tag == Team.RED) {
					if (redGoalFound()) {
						currentHeading = AngleFrom(tag);
						targetHeading = 0;
						headingPIDController.setCoefficients(new PIDFCoefficients(P, I, D, F));

					} else {
						currentHeading = follower.getPose().getHeading();
						targetHeading = calculateShotAngle(follower.getPose().getX(), follower.getPose().getY(), 0, 144);
						headingPIDController.setCoefficients(Constants.followerConstants.coefficientsHeadingPIDF);
					}
				}
			}
			@Override
			public void execute() {
				headingPIDController.updatePosition(-currentHeading);

				 headingError = targetHeading - currentHeading;
				headingError = Math.IEEEremainder(headingError, 2 * Math.PI);

				if (Math.abs(headingError) < Math.toRadians(7)) {
					headingCorrection = 0;
				} else {
					headingPIDController.setTargetPosition(targetHeading);
					headingCorrection = headingPIDController.run();
				}
				teleOpDrive.TeleopDrive(follower,0,0, headingCorrection);
			}
			@Override
			public boolean isFinished(){return Math.abs(headingError) < Math.toRadians(7);}
		};
	}
}