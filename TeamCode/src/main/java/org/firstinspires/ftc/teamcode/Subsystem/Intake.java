package org.firstinspires.ftc.teamcode.Subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

@Configurable
public class Intake extends SubsystemBase {
    private final DcMotorEx intake;

    public static final double IN_SPEED = 1;
    public static final double OUT_SPEED = -1;
    public static final double SLOW_SPEED = 0.5;
    public static final double SLOW_SPEED_OUT = -0.1;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public Command In() {
        return new InstantCommand(
                () -> intake.setPower(IN_SPEED), this
        );
    }

    public Command Slow() {
        return new InstantCommand(
                () -> intake.setPower(SLOW_SPEED), this
        );
    }

    public Command SlowOut() {
        return new InstantCommand(
                () -> intake.setPower(SLOW_SPEED_OUT), this
        );
    }

    public Command Out() {
        return new InstantCommand(
                () -> intake.setPower(OUT_SPEED), this
        );
    }

    public Command Stop() {
        return new InstantCommand(
                () -> intake.setPower(0), this
        );
    }

    public double getPower() {
        return intake.getPower();
    }
}