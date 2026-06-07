package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.Subsystem.Spindexer;


@TeleOp
public class TestSpindexer extends OpMode {
	private CommandScheduler scheduler;
	private Spindexer spindexer;
	private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
	private boolean spindexerPressed = false;

	@Override
	public void init() {
		spindexer = new Spindexer(hardwareMap);
		scheduler = CommandScheduler.getInstance();
		spindexer.zeroSpindexer();
	}

	@Override
	public void loop() {
		if ((gamepad1.a) && !spindexerPressed) {
			scheduler.schedule(spindexer.NewNextTarget());
			spindexerPressed = true;
		} else if ((!gamepad1.a) && spindexerPressed) {
			spindexerPressed = false;
		}


		spindexer.Telemetry(panelsTelemetry);
		scheduler.run();
		panelsTelemetry.update();
	}
}
