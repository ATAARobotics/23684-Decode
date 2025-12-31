package org.firstinspires.ftc.teamcode.Subsystem;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Utills.FeedForwardController;
import org.firstinspires.ftc.teamcode.Utills.PIDFController;

public class Shooter {
    // --- PID Controller Constants ---
    public static double UPPER_P = 0.0024, UPPER_I = 0, UPPER_D = 0;
    public static double LOWER_P = 0.001, LOWER_I = 0, LOWER_D = 0;

    // --- Feedforward Constants ---
    public static double UPPER_KS = 0.15, UPPER_KV = 0.000074;
    public static double LOWER_KS = 0.3, LOWER_KV = 0.00017;

    // --- Motor Power Constants ---
    public static double STOP_POWER = 0.0;

    // --- RPM & Control Constants ---
    public static double RPM_TOLERANCE = 50; // 100 before
    public static double TICKS_PER_REVOLUTION = 28.0;

    // --- Pre-calculated constants ---
    private static final double RPM_CONVERSION = 60.0 / TICKS_PER_REVOLUTION;
    private static final double HALF_DIVISOR = 0.5;
    public static double AUDIENCE_RPM = 2310;

    // --- Motor Offsets ---
    public static double UPPER_OFFSET = 0.0;
    public static double LOWER_OFFSET = 0.0;

    private DcMotorEx upperShooter;
    private DcMotorEx lowerShooter;

    // Public State
    public double averageRPM = 0.0;
    public double upperRPM = 0.0;
    public double lowerRPM = 0.0;

    // Acceleration tracking (kept if you need it for tuning, otherwise unused)
    public double upperAcceleration = 0.0;
    public double lowerAcceleration = 0.0;
    public double averageAcceleration = 0.0;

    private long lastNanoTime = 0;
    private double lastUpperVelocity = 0.0;
    private double lastLowerVelocity = 0.0;

    private PIDFController upperController;
    private PIDFController lowerController;
    private FeedForwardController upperFF;
    private FeedForwardController lowerFF;



    public Shooter(HardwareMap hardwareMap) {
        upperShooter = hardwareMap.get(DcMotorEx.class, "upperShooter");
        lowerShooter = hardwareMap.get(DcMotorEx.class, "lowerShooter");

        upperShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lowerShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        upperShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lowerShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        upperController = new PIDFController(UPPER_P, UPPER_I, UPPER_D, 0);
        lowerController = new PIDFController(LOWER_P, LOWER_I, LOWER_D, 0);
        upperFF = new FeedForwardController(UPPER_KS, UPPER_KV, 0);
        lowerFF = new FeedForwardController(LOWER_KS, LOWER_KV, 0);

    }

    private void updateRPM(long nanoTime) {
        double upperVelocity = upperShooter.getVelocity();
        double lowerVelocity = lowerShooter.getVelocity();

        upperRPM = upperVelocity * RPM_CONVERSION;
        lowerRPM = lowerVelocity * RPM_CONVERSION;
        averageRPM = (upperRPM + lowerRPM) * HALF_DIVISOR;

        if (lastNanoTime != 0) {
            double timeElapsedSeconds = (nanoTime - lastNanoTime) / 1_000_000_000.0;
            if (timeElapsedSeconds > 0) {
                upperAcceleration = (upperVelocity - lastUpperVelocity) / timeElapsedSeconds;
                lowerAcceleration = (lowerVelocity - lastLowerVelocity) / timeElapsedSeconds;
                averageAcceleration = (upperAcceleration + lowerAcceleration) * HALF_DIVISOR;
            }
        }
        lastNanoTime = nanoTime;
        lastUpperVelocity = upperVelocity;
        lastLowerVelocity = lowerVelocity;
    }

    public boolean isAtTargetRPM(double upperTarget, double lowerTarget) {
        // Check Upper: Must be a valid target (>100) AND within tolerance
        boolean upperReady = (upperTarget >= 100) && (Math.abs(upperRPM - upperTarget) <= RPM_TOLERANCE);

        // Check Lower: Must be a valid target (>100) AND within tolerance
        boolean lowerReady = (lowerTarget >= 100) && (Math.abs(lowerRPM - lowerTarget) <= RPM_TOLERANCE);

        return upperReady && lowerReady;
    }

    private void updateMotors(double upperTarget, double lowerTarget, TelemetryPacket packet) {
        double upperPower, lowerPower;

        // UPPER MOTOR
        if (upperTarget < 100) {
            upperPower = STOP_POWER;
        } else {
            // Only calculate PID/FF when we actually want to move
            double pid = upperController.getOutput(upperRPM, upperTarget);
            double ff = upperFF.calculate(upperTarget, 0);
            upperPower = ff + pid + UPPER_OFFSET;
        }

        // LOWER MOTOR
        if (lowerTarget < 100) {
            lowerPower = STOP_POWER;
        } else {
            double pid = lowerController.getOutput(lowerRPM, lowerTarget);
            double ff = lowerFF.calculate(lowerTarget, 0);
            lowerPower = ff + pid + LOWER_OFFSET;
        }

        upperShooter.setPower(upperPower);
        lowerShooter.setPower(lowerPower);

        // Standardized Telemetry - only send if packet is provided
        if (packet != null) {
            packet.put("Shooter Upper Target", upperTarget);
            packet.put("Shooter Lower Target", lowerTarget);
            packet.put("Shooter Upper RPM", upperRPM);
            packet.put("Shooter Lower RPM", lowerRPM);
            packet.put("Upper Power", upperPower);
            packet.put("Lower Power", lowerPower);
        }
    }

    public Command ToTarget(double upperTarget, double lowerTarget){
        return new CommandBase() {
            @Override
            public void execute() {
                updateRPM(System.nanoTime());
                updateMotors(upperTarget, lowerTarget, null);
            }
            @Override
            public boolean isFinished() {
                return isAtTargetRPM(upperTarget, lowerTarget);
            }
        };
    }

    public Command MindlessToTarget(double upperTarget, double lowerTarget){
        return new CommandBase() {
            @Override
            public void execute() {
                updateRPM(System.nanoTime());
                updateMotors(upperTarget, lowerTarget, null);
            }
            @Override
            public boolean isFinished() {
                return false;
            }
        };
    }
    public Command ToTarget(double upperTarget, double lowerTarget,TelemetryPacket packet){
        return new CommandBase() {
            @Override
            public void execute() {
                updateRPM(System.nanoTime());
                updateMotors(upperTarget, lowerTarget, packet);
            }
            @Override
            public boolean isFinished() {
                return isAtTargetRPM(upperTarget, lowerTarget);
            }
        };
    }


    public Command Stop(){
        return new CommandBase() {
            @Override
            public void execute() {
                upperShooter.setPower(STOP_POWER);
                lowerShooter.setPower(STOP_POWER);
            }
            @Override
            public boolean isFinished() {
                return true;
            }
        };
    }


}
