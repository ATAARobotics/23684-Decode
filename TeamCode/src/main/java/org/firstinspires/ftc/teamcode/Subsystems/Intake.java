package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Intake {

	// Motor power constants
	public static double IN_POWER = 1.0;
	public static double SLOW_POWER = 0.3;
	public static double OUT_POWER = -1.0;
	public static double STOP_POWER = 0.0;
	private static Intake instance = null;
	private DcMotor intake;

	private Intake() {
	}

	public static void initialize(HardwareMap hardwareMap) {
		instance = new Intake();
		instance.intake = hardwareMap.get(DcMotor.class, "intake");
	}

	public static Intake getInstance() {
		if (instance == null) {
			throw new IllegalStateException("Intake not initialized. Call initialize(hardwareMap) first.");
		}
		return instance;
	}

	public static void shutdown() {
		// No cleanup needed currently
	}

	/**
	 * Returns an InstantAction that runs the intake motor forward.
	 * This action completes immediately after setting power.
	 *
	 * @return Action that starts intake motor forward
	 */
	public Action in() {
		return new InstantAction(() -> intake.setPower(IN_POWER));
	}

	/**
	 * Returns an InstantAction that runs the intake motor forward slowly.
	 * This action completes immediately after setting power.
	 *
	 * @return Action that starts intake motor forward slowly
	 */
	public Action slow() {
		return new InstantAction(() -> intake.setPower(SLOW_POWER));
	}

	/**
	 * Returns an InstantAction that runs the intake motor backward.
	 * This action completes immediately after setting power.
	 *
	 * @return Action that starts intake motor backward
	 */
	public Action out() {
		return new InstantAction(() -> intake.setPower(OUT_POWER));
	}

	/**
	 * Returns an InstantAction that stops the intake motor.
	 * This action completes immediately after setting power to zero.
	 *
	 * @return Action that stops intake motor
	 */
	public Action stop() {
		return new InstantAction(() -> intake.setPower(STOP_POWER));
	}
}
