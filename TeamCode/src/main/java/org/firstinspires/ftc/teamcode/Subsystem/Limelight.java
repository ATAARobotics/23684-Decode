package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Utils.DistanceFromTag;

import java.util.ArrayList;
import java.util.List;

public class Limelight extends SubsystemBase {
	private final Limelight3A limelight;
	private final GoBildaPinpointDriver pinpoint;
	private double yawOffset = 0;

	public Pose3D botPose = new Pose3D(new Position(DistanceUnit.INCH, 0, 0, 0, 0), null);
	public boolean validResult = false;
	public List<LLResultTypes.FiducialResult> tags = new ArrayList<>();

	private static final int MAX_TAGS = 8;
	public final List<DistanceFromTag> distanceFromTags = new ArrayList<>();
	private final List<DistanceFromTag> pool = new ArrayList<>();

	private static final double METERS_TO_INCHES = 39.3701;

	public Limelight(HardwareMap hardwareMap) {
		limelight = hardwareMap.get(Limelight3A.class, "limelight");
		pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

		// Pre-fill the pool
		for (int i = 0; i < MAX_TAGS; i++) {
			pool.add(new DistanceFromTag(0, 0));
		}

		// Initializing pinpoint settings as per your configuration
		pinpoint.setOffsets(-177.8, -63.5, DistanceUnit.MM);
		pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
		pinpoint.setEncoderDirections(
				GoBildaPinpointDriver.EncoderDirection.REVERSED,
				GoBildaPinpointDriver.EncoderDirection.REVERSED
		);
	}

	public void init() {
		init(0);
	}

	public void init(double yawOffset) {
		this.yawOffset = yawOffset;
		limelight.pipelineSwitch(0);
		limelight.start();
	}

	public Command start() {
		return new InstantCommand(this::init, this);
	}

	public Command start(double yawOffset) {
		return new InstantCommand(() -> this.init(yawOffset), this);
	}

	public void updateData() {
		// 1. Update Orientation
		limelight.updateRobotOrientation(pinpoint.getHeading(AngleUnit.DEGREES) + yawOffset);

		// 2. Get Result
		LLResult llResult = limelight.getLatestResult();

		// 3. Reset State
		distanceFromTags.clear();
		validResult = false;

		if (llResult != null && llResult.isValid()) {
			tags = llResult.getFiducialResults();

			// Local primitives to track robot position for distance math
			double robotX = 0, robotY = 0;

			// 4. Process Pose (MegaTag 2)
			if (llResult.getBotposeTagCount() > 0) {
				validResult = true;
				Pose3D rawPose = llResult.getBotpose_MT2();
				Position rawPos = rawPose.getPosition();

				robotX = rawPos.x * METERS_TO_INCHES;
				robotY = rawPos.y * METERS_TO_INCHES;
				double robotZ = rawPos.z * METERS_TO_INCHES;

				botPose = new Pose3D(
						new Position(DistanceUnit.INCH, robotX, robotY, robotZ, rawPos.acquisitionTime),
						rawPose.getOrientation()
				);
			}

			// 5. Process Distances (Logic moved outside botposeTagCount check)
			int resultCount = Math.min(tags.size(), MAX_TAGS);
			for (int i = 0; i < resultCount; i++) {
				LLResultTypes.FiducialResult tag = tags.get(i);
				int id = (int) tag.getFiducialId();

				double tagX, tagY;
				boolean knownTag = true;

				switch (id) {
					case 20: tagX = -58.3727; tagY = -55.6425; break;
					case 24: tagX = -58.3727; tagY = 55.6425; break;
					default:
						knownTag = false;
						tagX = 0; tagY = 0;
				}

				if (knownTag && validResult) {
					double dx = tagX - robotX;
					double dy = tagY - robotY;
					double distance = Math.sqrt(dx * dx + dy * dy);

					// POOLING: Grab an object from the pool instead of 'new'
					DistanceFromTag dft = pool.get(i);
					dft.setId(id);           // Ensure your DistanceFromTag class has setters
					dft.setDistance(distance);
					distanceFromTags.add(dft);
				}
			}
		}
	}

	public Command update() {
		return new InstantCommand(this::updateData, this);
	}
}