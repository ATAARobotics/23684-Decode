package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.teamcode.Utilities.BallColor;
import org.firstinspires.ftc.teamcode.Utilities.PIDFController;

@Config
public class Spindexer {
	// 360 ticks per 360 degrees for the through-bore encoder
	public static double DEGREES_PER_TICK = (double) 360 / 8192;
	public static double SPIN_POWER = 1;
	private static Spindexer instance = null;
	// Store detected ball colors for each slot (0, 1, 2)
	private final BallColor[] ballColors = new BallColor[3];
	private CRServo spindexerLeft;
	private CRServo spindexerRight;
	private DcMotorEx spindexerEncoder;
	private int currentSlot = 0; // Track the current slot
	private double slotError = 0;
	private int targetSlot = 0;
	private double currentPower = 0;
	private double startTime = 0;
	public enum Status {
		NOT_AT_TARGET,
		ALMOST_AT_TARGET,
		AT_TARGET
	}
	public Status status = Status.NOT_AT_TARGET;

	PIDFController spindexerPIDF;
	public double p = 0.001;


	private Spindexer() {
		// Initialize all slots as EMPTY
		for (int i = 0; i < 3; i++) {
			ballColors[i] = BallColor.EMPTY;
		}
	}

	public static void initialize(HardwareMap hardwareMap) {
		instance = new Spindexer();
		instance.spindexerLeft = hardwareMap.get(CRServo.class, "spindexerLeft");
		instance.spindexerRight = hardwareMap.get(CRServo.class, "spindexerRight");
		instance.spindexerEncoder = hardwareMap.get(DcMotorEx.class, "intake");
		instance.spindexerEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
		instance.spindexerEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		instance.spindexerEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		instance.spindexerPIDF = new PIDFController(0.001, 0, 0, 0);
	}

	public static Spindexer getInstance() {
		if (instance == null) {
			throw new IllegalStateException("Spindexer not initialized. Call initialize(hardwareMap, elapsedTime) first.");
		}
		return instance;
	}

	public static void shutdown() {
		// No cleanup needed currently
	}

	/**
	 * Gets the encoder position in degrees.
	 */
	public double getPosition() {
		return spindexerEncoder.getCurrentPosition() * DEGREES_PER_TICK;
	}

	/**
	 * Updates the calculations of the current position, the current slot, and the error of the current slot.
	 */
	public void updateSlot() {
		double currentPosition = ((getPosition() % 360) + 360) % 360;

		if (currentPosition >= 300 || currentPosition < 60) {
			currentSlot = 0;
			if (currentPosition >= 300) {
				slotError = currentPosition - 360;
			} else {
				slotError = currentPosition;
			}
		} else if (currentPosition >= 60 && currentPosition < 180) {
			currentSlot = 1;
			slotError = currentPosition - 120;
		} else if (currentPosition >= 180 && currentPosition < 300) {
			currentSlot = 2;
			slotError = currentPosition - 240;
		}
	}

	public void update() {
		updateSlot();

		if (Math.abs(slotError) > 10) {
			status = Status.NOT_AT_TARGET;
			startTime = 0;
		}

		if (status != Status.AT_TARGET) {
			if (currentPower != SPIN_POWER) {
				spindexerLeft.setPower(SPIN_POWER);
				spindexerRight.setPower(SPIN_POWER);
				currentPower = SPIN_POWER;
			}

			boolean isSettling = (status == Status.ALMOST_AT_TARGET);

			// Allow slightly larger error (e.g. 7) if we are already settling to prevent reset
			double threshold = isSettling ? 7 : 5;

			if (currentSlot == targetSlot && Math.abs(slotError) < threshold) {
				status = Status.ALMOST_AT_TARGET;

				if (startTime == 0) {
					startTime = System.nanoTime();
				}

				if (System.nanoTime() - startTime > 400_000_000L) {
					spindexerLeft.setPower(0);
					spindexerRight.setPower(0);
					currentPower = 0;
					startTime = 0;
					status = Status.AT_TARGET;
				}
			} else {
				status = Status.NOT_AT_TARGET;
				startTime = 0;
			}
		}
	}

	public Action setTarget(int slot) {
		return new InstantAction(() -> targetSlot = slot);
	}

	public Action NextSlot() {
		return new Action() {
			boolean initialized = false;
			double target = spindexerEncoder.getCurrentPosition() + ((double) 8192 /3);

			@Override
			public boolean run(@NonNull TelemetryPacket telemetryPacket) {

				spindexerPIDF.setPID(p,0,0,0);
				double power = spindexerPIDF.getOutput(spindexerEncoder.getCurrentPosition(), target);
					spindexerLeft.setPower(power);
					spindexerRight.setPower(power);



					double error = Math.abs(spindexerEncoder.getCurrentPosition() - target);



				return error < 100;
			}

		};
	}

	public Action next() {
		return packet -> {
			boolean checked = false;
			double target = 0;
			if (!checked) {
				 target = spindexerEncoder.getCurrentPosition() + ((double) 8192 / 3);
				checked = true;
			}else {
				if (spindexerEncoder.getCurrentPosition() >= target) {
					spindexerLeft.setPower(0);
					spindexerRight.setPower(0);
				} else {
					spindexerLeft.setPower(SPIN_POWER);
					spindexerRight.setPower(SPIN_POWER);
				}
			}
			return spindexerEncoder.getCurrentPosition() <= target;
		};
	}


	public Action TwoSlots(){
		return new SequentialAction(
				NextSlot(),
				NextSlot()
		);
	}

	public Action toSlot(int slot) {
		return new Action() {
			boolean initialized = false;

			@Override
			public boolean run(@NonNull TelemetryPacket telemetryPacket) {
				if (initialized) {
					return !status.equals(Status.AT_TARGET);
				} else {
					initialized = true;
					setTarget(slot).run(new TelemetryPacket());
					return true;
				}
			}
		};
	}

	public BallColor getBallColor(int slotIndex) {
		if (slotIndex >= 0 && slotIndex < 3) {
			return ballColors[slotIndex];
		}
		return BallColor.UNKNOWN;
	}

	public void setBallColor(int slotIndex, BallColor color) {
		if (slotIndex >= 0 && slotIndex < 3) {
			ballColors[slotIndex] = color;
		}
	}

	/**
	 * Force set the spindexer power to a value between -1 and 1.
	 * Clamps the power to the valid range.
	 */
	public InstantAction setDirectPower(double power) {
		return new InstantAction(() -> {
			// Clamp power between -1 and 1
			double clampedPower = Math.max(-1.0, Math.min(1.0, power));
			spindexerLeft.setPower(clampedPower);
			spindexerRight.setPower(clampedPower);
		});
	}
}
