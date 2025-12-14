package org.firstinspires.ftc.teamcode.LifecycleManagementUtilities;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.ColorDetector;
import org.firstinspires.ftc.teamcode.Subsystems.DistanceDetector;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.RGBIndicator;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystems.TouchDetector;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;

/**
 * Initializes all subsystems in the correct order.
 * Call this in your OpMode's init() method.
 */
public class HardwareInitializer {
	public static void initialize(HardwareMap hardwareMap) {
		// Initialize subsystems with no dependencies first
		ColorDetector.initialize(hardwareMap);
		Transfer.initialize(hardwareMap);
		Shooter.initialize(hardwareMap);
		Intake.initialize(hardwareMap);
		RGBIndicator.initialize(hardwareMap);
		DistanceDetector.initialize(hardwareMap);

		// Initialize Spindexer last (depends on some other subsystems)
		Spindexer.initialize(hardwareMap);
	}
}
