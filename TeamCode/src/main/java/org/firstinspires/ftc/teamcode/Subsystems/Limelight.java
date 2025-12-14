package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Utilities.DistanceFromTag;

import java.util.ArrayList;
import java.util.List;

public class Limelight {
	private final Limelight3A limelight;
	private final GoBildaPinpointDriver pinpoint;
	private double yawOffset = 0;

	// Public fields
	public Pose3D botPose;
	public boolean validResult = false;
	public List<LLResultTypes.FiducialResult> tags;

	// Initialize once to avoid null checks.
	// This list is reused every loop to prevent memory allocation.
	public final List<DistanceFromTag> distanceFromTags = new ArrayList<>();

	private static final double METERS_TO_INCHES = 39.3701;

	public Limelight(HardwareMap hardwareMap) {
		limelight = hardwareMap.get(Limelight3A.class, "limelight");

		// Pinpoint setup
		pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
		pinpoint.setOffsets(-177.8, -63.5, DistanceUnit.MM);
		pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
		pinpoint.setEncoderDirections(
				GoBildaPinpointDriver.EncoderDirection.REVERSED,
				GoBildaPinpointDriver.EncoderDirection.REVERSED
		);
	}

	public void start() {
		start(0);
	}

	public void start(double yawOffset) {
		this.yawOffset = yawOffset;
		limelight.pipelineSwitch(0);
		limelight.start();
	}

	public void update() {
		// 1. Update Orientation (Fast hardware read)
		limelight.updateRobotOrientation(pinpoint.getHeading(AngleUnit.DEGREES) + yawOffset);

		// 2. Get Result
		LLResult llResult = limelight.getLatestResult();

		// 3. Reset State
		distanceFromTags.clear();
		validResult = false;

		// 4. Validate and Process
		if (llResult != null && llResult.isValid()) {
			tags = llResult.getFiducialResults();

			if (llResult.getBotposeTagCount() > 0) {
				validResult = true;

				// OPTIMIZATION: Manually convert units to avoid 'toUnit()' object allocation
				// MT2 usually returns meters. We apply the conversion to primitives directly.
				Pose3D rawPose = llResult.getBotpose_MT2();
				Position rawPos = rawPose.getPosition();

				double robotX = rawPos.x * METERS_TO_INCHES;
				double robotY = rawPos.y * METERS_TO_INCHES;
				double robotZ = rawPos.z * METERS_TO_INCHES;

				// Update public pose
				botPose = new Pose3D(
						new Position(DistanceUnit.INCH, robotX, robotY, robotZ, rawPos.acquisitionTime),
						rawPose.getOrientation()
				);

				// 5. Calculate Distances
				int tagCount = tags.size();
				for (int i = 0; i < tagCount; i++) {
					int id = tags.get(i).getFiducialId();

					double tagX, tagY;
					boolean calculate = true;

					// Hardcoded positions (Inches)
					switch (id) {
						case 20: // BlueTarget
							tagX = -58.3727;
							tagY = -55.6425;
							break;
						case 24: // RedTarget
							tagX = -58.3727;
							tagY = 55.6425;
							break;
						default:
							calculate = false;
							tagX = 0; tagY = 0;
					}

					if (calculate) {
						double dx = tagX - robotX;
						double dy = tagY - robotY;
						distanceFromTags.add(new DistanceFromTag(id, Math.sqrt(dx * dx + dy * dy)));
					}
				}
			}
		}
	}
}
