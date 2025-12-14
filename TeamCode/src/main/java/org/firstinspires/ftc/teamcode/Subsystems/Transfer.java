package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Transfer {
	// Servo power constants
	public static double FORWARD_POWER = 1.0;
	public static double BACKWARD_POWER = -1.0;
	public static double STOP_POWER = 0.0;
	public static double RIGHT_SERVO_POWER_SCALE = 0.8;
	private static Transfer instance = null;
	private CRServo transferLeft;
	private CRServo transferRight;
	private CRServo intakeDoorLeft;
	private CRServo intakeDoorRight;

	private Transfer() {
	}

	public static void initialize(HardwareMap hardwareMap) {
		instance = new Transfer();
		instance.transferLeft = hardwareMap.get(CRServo.class, "transferLeft");
		instance.transferRight = hardwareMap.get(CRServo.class, "transferRight");
		instance.intakeDoorLeft = hardwareMap.get(CRServo.class, "intakeDoorLeft");
		instance.intakeDoorRight = hardwareMap.get(CRServo.class, "intakeDoorRight");
		instance.setTransferDirections();
		instance.setIntakeDoorDirections();
	}

	public static Transfer getInstance() {
		if (instance == null) {
			throw new IllegalStateException("Transfer not initialized. Call initialize(hardwareMap) first.");
		}
		return instance;
	}

	public static void shutdown() {
		// No cleanup needed currently
	}

	// Direction setter methods
	private void setTransferDirections() {
		transferLeft.setDirection(DcMotorSimple.Direction.FORWARD);
		transferRight.setDirection(DcMotorSimple.Direction.REVERSE);
	}

	private void setIntakeDoorDirections() {
		intakeDoorRight.setDirection(DcMotorSimple.Direction.FORWARD);
		intakeDoorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
	}

	// Actions for transfer servos

	/**
	 * Returns an InstantAction that moves the transfer servos forward.
	 * This action completes immediately after setting servo power.
	 *
	 * @return Action that starts transfer servos forward
	 */
	public Action transferForward() {
		return new InstantAction(() -> {
			setTransferDirections();
			transferLeft.setPower(FORWARD_POWER);
			transferRight.setPower(FORWARD_POWER * RIGHT_SERVO_POWER_SCALE);
		});
	}

	/**
	 * Returns an InstantAction that moves the transfer servos backward.
	 * This action completes immediately after setting servo power.
	 *
	 * @return Action that starts transfer servos backward
	 */
	public Action transferBackward() {
		return new InstantAction(() -> {
			setTransferDirections();
			transferLeft.setPower(BACKWARD_POWER);
			transferRight.setPower(BACKWARD_POWER * RIGHT_SERVO_POWER_SCALE);
		});
	}

	/**
	 * Returns an InstantAction that stops the transfer servos.
	 * This action completes immediately after setting power to zero.
	 *
	 * @return Action that stops transfer servos
	 */
	public Action transferStop() {
		return new InstantAction(() -> {
			setTransferDirections();
			transferLeft.setPower(STOP_POWER);
			transferRight.setPower(STOP_POWER);
		});
	}

	// Actions for intake door servos

	/**
	 * Returns an InstantAction that moves the intake door servos forward (opens).
	 * This action completes immediately after setting servo power.
	 *
	 * @return Action that opens intake door
	 */
	public Action intakeDoorForward() {
		return new InstantAction(() -> {
			setIntakeDoorDirections();
			intakeDoorLeft.setPower(FORWARD_POWER);
			intakeDoorRight.setPower(FORWARD_POWER);
		});
	}

	/**
	 * Returns an InstantAction that moves the intake door servos backward (closes).
	 * This action completes immediately after setting servo power.
	 *
	 * @return Action that closes intake door
	 */
	public Action intakeDoorBackward() {
		return new InstantAction(() -> {
			setIntakeDoorDirections();
			intakeDoorLeft.setPower(BACKWARD_POWER);
			intakeDoorRight.setPower(BACKWARD_POWER);
		});
	}

	/**
	 * Returns an InstantAction that stops the intake door servos.
	 * This action completes immediately after setting power to zero.
	 *
	 * @return Action that stops intake door
	 */
	public Action intakeDoorStop() {
		return new InstantAction(() -> {
			setIntakeDoorDirections();
			intakeDoorLeft.setPower(STOP_POWER);
			intakeDoorRight.setPower(STOP_POWER);
		});
	}

	// Direct servo control methods for continuous operation

	/**
	 * Directly sets the intake door servo power (for continuous control).
	 *
	 * @param power The power to set (-1.0 to 1.0)
	 */
	public void setIntakeDoorPower(double power) {
		setIntakeDoorDirections();
		intakeDoorLeft.setPower(power);
		intakeDoorRight.setPower(power);
	}
}
