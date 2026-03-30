package org.firstinspires.ftc.teamcode.OpModes.Test;

import static org.firstinspires.ftc.teamcode.Subsystem.Shooter.HALF_DIVISOR;
import static org.firstinspires.ftc.teamcode.Subsystem.Shooter.RPM_CONVERSION;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystem.Shooter;

@TeleOp(name = "Shooter No PID", group = "Test")
public class ShooterNoPID extends OpMode {
    DcMotorEx upperShooter;
    DcMotorEx lowerShooter;

    double averageRPM = 0.0;
    double upperRPM = 0.0;
    double lowerRPM = 0.0;


    private void updateRPM() {
        double upperVelocity = upperShooter.getVelocity();
        double lowerVelocity = lowerShooter.getVelocity();

        upperRPM = upperVelocity * Shooter.RPM_CONVERSION;
        lowerRPM = lowerVelocity * Shooter.RPM_CONVERSION;
        averageRPM = (upperRPM + lowerRPM) * Shooter.HALF_DIVISOR;
    }

    @Override
    public void init() {
        upperShooter = hardwareMap.get(DcMotorEx.class, "upperShooter");
        lowerShooter = hardwareMap.get(DcMotorEx.class, "lowerShooter");

        upperShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lowerShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        upperShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lowerShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        updateRPM();

       upperShooter.setPower(1);
       lowerShooter.setPower(1);

       telemetry.addData("Upper RPM", upperRPM);
       telemetry.addData("Lower RPM", lowerRPM);
       telemetry.addData("Average RPM", averageRPM);
       telemetry.update();

    }


}
