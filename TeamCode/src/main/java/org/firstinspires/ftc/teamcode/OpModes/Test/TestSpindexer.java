package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.Subsystem.Spindexer;

@TeleOp
public class TestSpindexer extends OpMode {
	CommandScheduler scheduler;
	boolean spindexerPressed;
	private Spindexer spindexer;
	TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

	@Override
	public void init() {
		spindexer = new Spindexer(hardwareMap);
		scheduler = CommandScheduler.getInstance();
	}

	@Override
	public void loop() {
		if ((gamepad1.a) && !spindexerPressed) {
			scheduler.schedule(spindexer.NextTarget());
			spindexerPressed = true;
		} else if ((!gamepad1.a) && spindexerPressed) {
			spindexerPressed = false;
		}

		spindexer.Telemetry(panelsTelemetry);
		scheduler.run();
		panelsTelemetry.update();
	}
}
