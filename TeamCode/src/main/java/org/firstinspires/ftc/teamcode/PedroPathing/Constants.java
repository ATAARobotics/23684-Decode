package org.firstinspires.ftc.teamcode.PedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
	public static FollowerConstants followerConstants = new FollowerConstants()
			.mass(15.61)
			.forwardZeroPowerAcceleration(-39.618506846933805)
			.lateralZeroPowerAcceleration(-74.4028325085069)
			.translationalPIDFCoefficients(new PIDFCoefficients(0.4, 0, 0.025, 0.02))
			.headingPIDFCoefficients(new PIDFCoefficients(5, 0, 0.12, 0.018))
			.drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1, 0, 0.0005, 0, 0))
			.centripetalScaling(0.006);

	public static MecanumConstants driveConstants = new MecanumConstants()
			.maxPower(1)
			.rightFrontMotorName("frontRight")
			.rightRearMotorName("rearRight")
			.leftFrontMotorName("frontLeft")
			.leftRearMotorName("rearLeft")
			.leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
			.leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
			.rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
			.rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
			.xVelocity(73.0866293919681)
			.yVelocity(54.393549240793135);

	public static PinpointConstants localizerConstants = new PinpointConstants()
			.forwardPodY(-2.5)
			.strafePodX(-7.0)
			.distanceUnit(DistanceUnit.INCH)
			.hardwareMapName("pinpoint")
			.encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
			.forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
			.strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

	public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1.3, 1);

	public static Follower createFollower(HardwareMap hardwareMap) {
		return new FollowerBuilder(followerConstants, hardwareMap)
				.pathConstraints(pathConstraints)
				.mecanumDrivetrain(driveConstants)
				.pinpointLocalizer(localizerConstants)
				.build();
	}
}
