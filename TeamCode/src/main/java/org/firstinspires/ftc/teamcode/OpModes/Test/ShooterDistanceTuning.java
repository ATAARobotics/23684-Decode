package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;
import org.firstinspires.ftc.teamcode.Subsystem.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystem.Transfer;
import org.firstinspires.ftc.teamcode.Utils.ShootAngle;
import org.firstinspires.ftc.teamcode.Utils.Team;

import java.util.function.Supplier;

@Config
@Configurable
@TeleOp
public class ShooterDistanceTuning extends OpMode {
	public static double upperMotorRPM = 0;
	public static double lowerMotorRPM = 0;
	public static double spindexerSpeed = 0;
	public static Team team = Team.BLUE;

	CommandScheduler scheduler;
	Shooter shooter;
	Spindexer spindexer;
	Intake intake;
	Transfer transfer;
	Follower follower;

	protected boolean aButtonPressed = false;
	private Supplier<PathChain> pathChain;

	TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

	@Override
	public void init() {
		scheduler = CommandScheduler.getInstance();
		scheduler.setBulkReading(hardwareMap, LynxModule.BulkCachingMode.AUTO);
		shooter = new Shooter(hardwareMap);
		spindexer = new Spindexer(hardwareMap);
		intake = new Intake(hardwareMap);
		transfer = new Transfer(hardwareMap);
		follower = Constants.createFollower(hardwareMap);
		follower.setStartingPose(new Pose(63.450, 9, 270));

		scheduler.reset();
		scheduler.run();

		shooter.setTuningMode(true);

		transfer.runAutomaticTransfer = true;

		pathChain = () -> follower.pathBuilder()
				.addPath(new Path(new BezierLine(follower::getPose, new Pose(70.609, 103.834))))
				.setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(151.386), 0.8))
				.build();
	}

	@Override
	public void loop() {
		follower.update();
		handleDriveInput();

		shooter.updatePIDCoefficients();
		scheduler.schedule(shooter.SetTarget(upperMotorRPM, lowerMotorRPM));
		scheduler.schedule(spindexer.DirectPower(gamepad2.left_stick_y * spindexerSpeed));

		if (gamepad2.xWasPressed()) {
			scheduler.schedule(intake.In());
			scheduler.schedule(transfer.IntakeDoorOut());
		} else if (gamepad2.xWasReleased()) {
			scheduler.schedule(intake.Stop());
			scheduler.schedule(transfer.IntakeDoorStop());
		}

		if (gamepad2.bWasPressed()) {
			scheduler.schedule(intake.Out());
			scheduler.schedule(transfer.IntakeDoorIn());
		} else if (gamepad2.bWasReleased()) {
			scheduler.schedule(intake.Stop());
			scheduler.schedule(transfer.IntakeDoorStop());
		}

		panelsTelemetry.addData("Upper RPM", shooter.upperRPM);
		panelsTelemetry.addData("Lower RPM", shooter.lowerRPM);
		panelsTelemetry.addData("Upper Target", shooter.upperTarget);
		panelsTelemetry.addData("Lower Target", shooter.lowerTarget);

		panelsTelemetry.addData("xyz", follower.getPose());

		panelsTelemetry.update();

		scheduler.run();
		shooter.periodic();
	}

	private void handleDriveInput() {
		if (gamepad1.a && !aButtonPressed) {
			if (team == Team.RED) {
				follower.turnTo(ShootAngle.calculateShotAngle(follower.getPose().getX(), follower.getPose().getY(), 144, 144));
			} else if (team == Team.BLUE) {
				follower.followPath(pathChain.get(), true);
			}

			aButtonPressed = true;
		} else if (!gamepad1.a && aButtonPressed) {
			aButtonPressed = false;
		}

		if (!gamepad1.a) {
			if (!follower.isTeleopDrive()) {
				follower.startTeleOpDrive(true);
			}

			follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
		}
	}
}
