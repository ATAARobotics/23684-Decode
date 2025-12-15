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
	 * 
	 * @param sendTelemetry if true, sends telemetry to FTC Dashboard; if false, skips dashboard overhead
	 */
	public static void update(boolean sendTelemetry) {
		// TODO: Move Shooter PID control from run to update. Run should only set the target RPM.
//		Shooter.getInstance().update();

		if (sendTelemetry) {
			TelemetryPacket telemetryPacket = new TelemetryPacket();
			ColorDetector.getInstance().update().run(telemetryPacket);
			DistanceDetector.getInstance().update().run(telemetryPacket);
			FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);
		} else {
			// Still update sensors but without dashboard overhead
			ColorDetector.getInstance().update().run(null);
			DistanceDetector.getInstance().update().run(null);
		}
	}

	/**
	 * Updates all subsystems with telemetry enabled (default behavior for backward compatibility).
	 * Calls update(true).
	 */
	public static void update() {
		update(true);
	}
}
