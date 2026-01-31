package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utils.ShootAngle;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class TestDrive extends OpMode {
	public boolean checked = false;
	Path path;
	Pose holdpose;
	private Follower follower;
	private boolean automatedDrive;
	private Supplier<PathChain> pathChain;
	private TelemetryManager telemetryM;
	private boolean slowMode = false;
	private double slowModeMultiplier = 0.5;

	@Override
	public void init() {
		// follower = Constants.createFollower(hardwareMap);
		follower.setStartingPose(new Pose(72, 72, Math.toRadians(270)));
		holdpose = new Pose(0, 0, 0);
		follower.update();
		telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
		pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
				.addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
				.setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
				.build();
	}


	@Override
	public void start() {
		follower.startTeleopDrive();
	}

	@Override
	public void loop() {
		//Call this once per loop
		follower.update();
		telemetryM.update();

		if (!automatedDrive) {
			// Make the last parameter false for field-centric
			// In case the drivers want to use a "slowMode" you can scale the vectors

			// This is the normal version to use in the TeleOp
			if (!slowMode) follower.setTeleOpDrive(
					-gamepad1.left_stick_y,
					-gamepad1.left_stick_x,
					-gamepad1.right_stick_x,
					true // Robot Centric
			);

				// This is how it looks with slowMode on
			else follower.setTeleOpDrive(
					-gamepad1.left_stick_y * slowModeMultiplier,
					-gamepad1.left_stick_x * slowModeMultiplier,
					-gamepad1.right_stick_x * slowModeMultiplier,
					true // Robot Centric
			);

			if (Math.abs(gamepad1.left_stick_y) + Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.right_stick_x) == 0) {
				if (!checked) {
					holdpose = follower.getPose();
					checked = true;
				}
				follower.holdPoint(holdpose);
			} else {
				checked = false;
			}
		}

		if (gamepad1.yWasPressed()) {
			follower.turnTo(ShootAngle.calculateShotAngle(follower.getPose().getX(), follower.getPose().getY(), 0, 144));
		}

		if (gamepad1.xWasPressed()) {
			path.setHeadingInterpolation(HeadingInterpolator.facingPoint(0, 144));
		}

		// Automated PathFollowing
		if (gamepad1.aWasPressed()) {
			follower.followPath(pathChain.get());
			automatedDrive = true;
		}

		// Stop automated following if the follower is done
		if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
			follower.startTeleopDrive();
			automatedDrive = false;
		}

		// Slow Mode
		if (gamepad1.rightBumperWasPressed()) {
			slowMode = !slowMode;
		}

		// Optional way to change slow mode strength
		if (gamepad1.xWasPressed()) {
			slowModeMultiplier += 0.25;
		}

		// Optional way to change slow mode strength
		if (gamepad2.yWasPressed()) {
			slowModeMultiplier -= 0.25;
		}

		telemetryM.debug("position", follower.getPose());
		telemetryM.debug("velocity", follower.getVelocity());
		telemetryM.debug("automatedDrive", automatedDrive);
	}
}