package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystem.Spindexer;


@TeleOp
public class DistanceSensorTest extends OpMode {

	private DistanceSensor distanceSensor;
	private Spindexer spindexer;
	private CommandScheduler scheduler;
	private boolean artifactFound = false;

	@Override
	public void init() {
		distanceSensor = hardwareMap.get(DistanceSensor.class, "intakeDistanceSensor");
		spindexer = new Spindexer(hardwareMap);
		scheduler = CommandScheduler.getInstance();
	}

	@Override
	public void loop() {
		scheduler.run();

		boolean triggeredDistance = distanceSensor.getDistance(DistanceUnit.CM) < 20;

		if (triggeredDistance && !artifactFound) {
			scheduler.schedule(new SequentialCommandGroup(new WaitCommand(100), spindexer.NextTarget()));
			artifactFound = true;
		}
		if (!triggeredDistance && artifactFound) {
			artifactFound = false;
		}

		telemetry.addData("Distance", distanceSensor.getDistance(DistanceUnit.CM));
		telemetry.addData("Artifact Found", artifactFound);
		telemetry.update();
	}
}
