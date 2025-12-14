package org.firstinspires.ftc.teamcode.LifecycleManagementUtilities;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.ColorDetector;
import org.firstinspires.ftc.teamcode.Subsystems.DistanceDetector;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystems.TouchDetector;

/**
 * Handles periodic updates for subsystems that require continuous processing.
 * Call this once per loop cycle in your OpMode.
 */
public class SubsystemUpdater {
	/**
	 * Updates all subsystems that require periodic updates.
	 * Must be called every loop cycle for proper PID control and sensor readings.
	 */
	public static void update() {
		Shooter.getInstance().updateRPM(System.nanoTime());

		TelemetryPacket telemetryPacket = new TelemetryPacket();
		ColorDetector.getInstance().update().run(telemetryPacket);
		DistanceDetector.getInstance().update().run(telemetryPacket);
		FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);
	}
}
