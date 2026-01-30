package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;
import org.firstinspires.ftc.teamcode.Subsystem.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystem.Transfer;
import org.firstinspires.ftc.teamcode.Utils.DistanceFromTag;

@Config
@Configurable
@TeleOp
public class ShooterDistanceTuning extends OpMode {
    CommandScheduler scheduler;

	Shooter shooter;
	Spindexer spindexer;
	Intake intake;
	Transfer transfer;

	GamepadEx operatorGamepad;
	TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    static double upperMotorRPM = 0;
	static double lowerMotorRPM = 0;
	static double spindexerSpeed = 0;

    @Override
    public void init() {
        scheduler = CommandScheduler.getInstance();
		scheduler.setBulkReading(hardwareMap, LynxModule.BulkCachingMode.AUTO);
        shooter = new Shooter(hardwareMap);
		spindexer = new Spindexer(hardwareMap);
		intake = new Intake(hardwareMap);
		transfer = new Transfer(hardwareMap);
		scheduler.reset();
		scheduler.run();

		shooter.setTuningMode(true);

		operatorGamepad = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
		shooter.updatePIDCoefficients();
		scheduler.schedule(shooter.SetTarget(upperMotorRPM, lowerMotorRPM));
		scheduler.schedule(spindexer.DirectPower(gamepad2.left_stick_y * spindexerSpeed));

		if (gamepad2.aWasPressed()) {
			scheduler.schedule(intake.In());
			scheduler.schedule(transfer.IntakeDoorOut());
		}

		if (gamepad2.bWasPressed()) {
			scheduler.schedule(intake.Out());
			scheduler.schedule(transfer.IntakeDoorIn());
		}

		if (gamepad2.xWasPressed()) {
			scheduler.schedule(transfer.TransferOut());
		}

		panelsTelemetry.addData("Upper RPM", shooter.upperRPM);
		panelsTelemetry.addData("Lower RPM", shooter.lowerRPM);
		panelsTelemetry.addData("Upper Target", shooter.upperTarget);
		panelsTelemetry.addData("Lower Target", shooter.lowerTarget);
//		panelsTelemetry.addData("Length of Limelight Distance From Tags", limelight.distanceFromTags.size());
//		for (DistanceFromTag distanceFromTag : limelight.distanceFromTags) {
//			panelsTelemetry.addData(String.format("Distance From Tag %d", distanceFromTag.getTag()), distanceFromTag.getDistance());
//		}

		panelsTelemetry.update();

		scheduler.run();
    }
}
