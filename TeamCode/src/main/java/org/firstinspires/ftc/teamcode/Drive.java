package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


@Config
public class Drive extends SubsystemBase {
	public static final PIDFCoefficients HEADING_PID_QUICK = new PIDFCoefficients(0.04, 0, 0.0022, 0.0005);
	public static final PIDFCoefficients FORWARD_PID_QUICK = new PIDFCoefficients(0.042, 0, 0.0053, 0.0041);
	public static final PIDFCoefficients STRAFE_PID_QUICK = new PIDFCoefficients(0.051, 0, 0, 0.006);
	public static final Motor.ZeroPowerBehavior zeroPowerBehavior = Motor.ZeroPowerBehavior.BRAKE;
	private static final AngleUnit ANGLE_UNIT = AngleUnit.DEGREES;
	private static final DistanceUnit DISTANCE_UNIT = DistanceUnit.INCH;
	private static final double STATIC_F_SENSITIVE = 0.005;
	private static final double TURN_SPEED = 3;
	private final MecanumDrive drivebase;
	private final GoBildaPinpointDriver pinpoint;
	private double x;
	private double y;
	private double heading;
	private double strafe;
	private double forward;
	private double turn;
	private double ffX;
	private double ffY;
	private double ffTurn;
	private double ffHeading;
	private Pose2D currentPosition;
	private Pose2D previousPosition;
	private double headingUnnormalizeMultiplier = 0;
	private Pose2D target = new Pose2D(DISTANCE_UNIT, 0, 0, ANGLE_UNIT, 0);

	public Drive(HardwareMap hardwareMap) {
		Motor motorFL = new Motor(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_312);
		motorFL.setZeroPowerBehavior(zeroPowerBehavior);

		Motor motorFR = new Motor(hardwareMap, "frontRight", Motor.GoBILDA.RPM_312);
		motorFR.setZeroPowerBehavior(zeroPowerBehavior);

		Motor motorBL = new Motor(hardwareMap, "rearLeft", Motor.GoBILDA.RPM_312);
		motorBL.setZeroPowerBehavior(zeroPowerBehavior);

		Motor motorBR = new Motor(hardwareMap, "rearRight", Motor.GoBILDA.RPM_312);
		motorBR.setZeroPowerBehavior(zeroPowerBehavior);

		drivebase = new MecanumDrive(motorFL, motorFR, motorBL, motorBR);
		drivebase.setMaxSpeed(1);

		pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

		this.pinpoint.setOffsets(-173.0, 156, DistanceUnit.MM); //TODO: Check these values
		this.pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
		this.pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
		reset();
	}

	public void reset() {
		pinpoint.resetPosAndIMU();
	}

	public void stop() {
		drivebase.stop();
	}

	public double unnormalizeHeading(double angle) {
		if (previousPosition == null) {
			previousPosition = currentPosition;
		}

		if (previousPosition.getHeading(AngleUnit.DEGREES) > 90 &&
				previousPosition.getHeading(AngleUnit.DEGREES) < 181 &&
				currentPosition.getHeading(AngleUnit.DEGREES) < -90 && currentPosition.getHeading(AngleUnit.DEGREES) > -181) {
			// Flipped from 180 to -180
			headingUnnormalizeMultiplier += 1;
		} else if (previousPosition.getHeading(AngleUnit.DEGREES) < -90 &&
				previousPosition.getHeading(AngleUnit.DEGREES) > -181 &&
				currentPosition.getHeading(AngleUnit.DEGREES) > 90 && currentPosition.getHeading(AngleUnit.DEGREES) < 181) {
			// Flipped from -180 to 180
			headingUnnormalizeMultiplier -= 1;
		}

		previousPosition = currentPosition;

		return angle + (headingUnnormalizeMultiplier * 360);
	}

	public double[] headingCalculations(double currentHeading, double targetHeading) {
		double unnormalizedHeading = unnormalizeHeading(currentHeading);

		// Checks if headings are within bounds
//        if (unnormalizedHeading < -180 || unnormalizedHeading > 180 || targetHeading < -180 || targetHeading > 180) {
//            throw new IllegalArgumentException("Angles must be between -180 and 180");
//        }

		double distanceDirect = Math.abs(currentHeading - targetHeading);

		double distanceTo180FromCurrent = Math.min(Math.abs(unnormalizedHeading - 180), Math.abs(unnormalizedHeading + 180));
		double distanceTo180FromTarget = Math.min(Math.abs(targetHeading - 180), Math.abs(targetHeading + 180));
		double distanceCrossing180 = distanceTo180FromCurrent + distanceTo180FromTarget;

		if (distanceCrossing180 < distanceDirect) {
			// Crossing 180 degrees is faster
			// Trick the PID controller to go faster
			if (targetHeading > unnormalizedHeading) {
				targetHeading -= 360;
			} else {
				targetHeading += 360;
			}
		}

		return new double[]{unnormalizedHeading, targetHeading};
	}

	public Pose2D getPosition() {
		return currentPosition;
	}

	public void setPosition(Pose2D pose) {
		pinpoint.setPosition(pose);
		currentPosition = pose;
	}

	public Command moveTo(double x, double y, double h) {
		return new MoveTo(x, y, h);
	}

	@Override
	public void periodic() {
		pinpoint.update();
		drivebase.driveFieldCentric(x, y, heading, currentPosition.getHeading(ANGLE_UNIT));
	}

	public void readSensors() {
		Pose2D pinpointPosition = pinpoint.getPosition();
		currentPosition = new Pose2D(DISTANCE_UNIT, pinpointPosition.getX(DISTANCE_UNIT), pinpointPosition.getY(DISTANCE_UNIT), ANGLE_UNIT, -pinpointPosition.getHeading(ANGLE_UNIT));
	}

	public void addTelemetry(TelemetryPacket pack) {
		pack.put("Current X", currentPosition.getX(DISTANCE_UNIT));
		pack.put("Current Y", currentPosition.getY(DISTANCE_UNIT));
		pack.put("Target X", target.getX(DISTANCE_UNIT));
		pack.put("Target Y", target.getY(DISTANCE_UNIT));
		pack.put("Target Heading", target.getHeading(ANGLE_UNIT));
		pack.put("Current Heading", unnormalizeHeading(currentPosition.getHeading(ANGLE_UNIT)));

		// Draw robot on field

		double circleRadius = 10;
		double lineLength = 10;

//        pack.field();
//        pack.fieldOverlay()
//                .drawImage(ConstantsPro.FieldURL, 0, 0, 144, 144)
//                .setStroke("green")
//                .strokeCircle(currentPosition.getY(DISTANCE_UNIT), -currentPosition.getX(DISTANCE_UNIT), circleRadius)
//                .setStroke("red")
//                .strokeLine(currentPosition.getY(DISTANCE_UNIT) + (circleRadius - lineLength) * Math.cos(-currentPosition.getHeading(AngleUnit.RADIANS)), -currentPosition.getX(DISTANCE_UNIT) + (circleRadius - lineLength) * Math.sin(-currentPosition.getHeading(AngleUnit.RADIANS)), currentPosition.getY(DISTANCE_UNIT) + circleRadius * Math.cos(-currentPosition.getHeading(AngleUnit.RADIANS)), -currentPosition.getX(DISTANCE_UNIT) + circleRadius * Math.sin(-currentPosition.getHeading(AngleUnit.RADIANS)));
	}

	public class HumanInputs extends CommandBase {
		private final PIDFController quickTurn;
		GamepadEx driver;

		public HumanInputs(GamepadEx driver) {
			this.driver = driver;
			addRequirements(Drive.this);

			quickTurn = new PIDController(0.035, HEADING_PID_QUICK.i, HEADING_PID_QUICK.d);
			quickTurn.setTolerance(1);
		}

		@Override
		public void execute() {
//            strafe = scaleInputs(driver.getRightX());
//            forward = scaleInputs(-driver.getRightY());
			strafe = -driver.getRightX();
			forward = -driver.getRightY();
			double leftX = driver.getLeftX();
			double DEAD_ZONE = 0.1;
			if (Math.abs(leftX) > DEAD_ZONE)
				target = new Pose2D(DISTANCE_UNIT, target.getX(DISTANCE_UNIT), target.getY(DISTANCE_UNIT), ANGLE_UNIT, target.getHeading(ANGLE_UNIT) - TURN_SPEED * leftX);
			if (driver.wasJustPressed(GamepadKeys.Button.DPAD_UP))
				target = new Pose2D(DISTANCE_UNIT, target.getX(DISTANCE_UNIT), target.getY(DISTANCE_UNIT), ANGLE_UNIT, 0);
			if (driver.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))
				target = new Pose2D(DISTANCE_UNIT, target.getX(DISTANCE_UNIT), target.getY(DISTANCE_UNIT), ANGLE_UNIT, 180);
			if (driver.wasJustPressed(GamepadKeys.Button.DPAD_LEFT))
				target = new Pose2D(DISTANCE_UNIT, target.getX(DISTANCE_UNIT), target.getY(DISTANCE_UNIT), ANGLE_UNIT, 90);
			if (driver.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT))
				target = new Pose2D(DISTANCE_UNIT, target.getX(DISTANCE_UNIT), target.getY(DISTANCE_UNIT), ANGLE_UNIT, -90);

			double[] headings = headingCalculations(currentPosition.getHeading(ANGLE_UNIT), target.getHeading(ANGLE_UNIT));
			double currentHeading = headings[0];
			double targetHeading = headings[1];
			turn = quickTurn.calculate(currentHeading, targetHeading);
			turn = quickTurn.calculate(currentPosition.getHeading(ANGLE_UNIT), target.getHeading(ANGLE_UNIT));

			if (turn > STATIC_F_SENSITIVE) ffTurn = HEADING_PID_QUICK.f;
			if (turn < -STATIC_F_SENSITIVE) ffTurn = -HEADING_PID_QUICK.f;
			turn += ffTurn;
			turn = -turn;

			// Left trigger for turbo mode
//            if (driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5){
//                turbo(true);
//            } else {
//                turbo(false);
//            }
		}

		public double scaleInputs(double input) {
			double DEAD_ZONE = 0.1;
			if (Math.abs(input) > DEAD_ZONE)
				return Math.pow(Math.abs(input), 2) * Math.signum(input);
			else
				return 0;
		}
	}

	public class MoveTo extends CommandBase {
		private final PIDFController quickStrafe;
		private final PIDFController quickForward;
		private final PIDFController quickTurn;

		public MoveTo(double x, double y, double h) {
			target = new Pose2D(DISTANCE_UNIT, x, y, ANGLE_UNIT, h);
			quickStrafe = new PIDController(STRAFE_PID_QUICK.p, STRAFE_PID_QUICK.i, STRAFE_PID_QUICK.d);
			quickForward = new PIDController(FORWARD_PID_QUICK.p, FORWARD_PID_QUICK.i, FORWARD_PID_QUICK.d);
			quickTurn = new PIDController(HEADING_PID_QUICK.p, HEADING_PID_QUICK.i, HEADING_PID_QUICK.d);
			quickStrafe.setTolerance(0.01);
			quickForward.setTolerance(0.01);
			quickTurn.setTolerance(1);
			quickStrafe.setSetPoint(target.getX(DISTANCE_UNIT));
			quickForward.setSetPoint(target.getY(DISTANCE_UNIT));
			quickTurn.setSetPoint(target.getHeading(ANGLE_UNIT));
			addRequirements(Drive.this);
		}

		@Override
		public void initialize() {
			double TURBO_FAST_SPEED = 1.0;
			drivebase.setMaxSpeed(TURBO_FAST_SPEED);
		}

		@Override
		public void execute() {
			x = quickStrafe.calculate(currentPosition.getX(DISTANCE_UNIT), target.getX(DISTANCE_UNIT));
			y = quickForward.calculate(currentPosition.getY(DISTANCE_UNIT), target.getY(DISTANCE_UNIT));
			heading = quickTurn.calculate(unnormalizeHeading(currentPosition.getHeading(ANGLE_UNIT)), target.getHeading(ANGLE_UNIT));

			// Custom Static Friction Calculations
			if (x > STATIC_F_SENSITIVE) ffX = STRAFE_PID_QUICK.f;
			if (x < -STATIC_F_SENSITIVE) ffX = -STRAFE_PID_QUICK.f;
			if (y > STATIC_F_SENSITIVE) ffY = FORWARD_PID_QUICK.f;
			if (y < -STATIC_F_SENSITIVE) ffY = -FORWARD_PID_QUICK.f;
			if (heading > STATIC_F_SENSITIVE) ffHeading = HEADING_PID_QUICK.f;
			if (heading < -STATIC_F_SENSITIVE) ffHeading = -HEADING_PID_QUICK.f;

			x += ffX;
			y += ffY;
			heading += ffHeading;

			x = -x;
			heading = -heading;
		}

		@Override
		public boolean isFinished() {
			// Check if target is reached
			return quickStrafe.atSetPoint() && quickForward.atSetPoint() && quickTurn.atSetPoint();
		}

		@Override
		public void end(boolean interrupted) {
			x = 0;
			y = 0;
			heading = 0;
			stop();
		}
	}
}