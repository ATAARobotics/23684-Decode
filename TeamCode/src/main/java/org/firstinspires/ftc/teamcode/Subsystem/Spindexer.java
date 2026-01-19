package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utils.FeedForwardController;
import org.firstinspires.ftc.teamcode.Utils.PIDFController;
import org.firstinspires.ftc.teamcode.Utils.SpindexerPosition;

public class Spindexer extends SubsystemBase {
	public final DcMotor spindexerMotor;

    PIDFController spidexerPIDF;

    private double prevtarget = 0;

    private double P = 0, I = 0, D = 0, F = 0;

    public static boolean TUNING_MODE = false;



    public Spindexer(HardwareMap hardwareMap) {
        spindexerMotor = hardwareMap.get(DcMotor.class, "spindexerMotor");
        spindexerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        spidexerPIDF = new PIDFController(P,I,D,F);

    }

    public void periodic(){
        updatePIDCoefficients();
    }

    public void updatePIDCoefficients() {
        if (TUNING_MODE) {
           spidexerPIDF = new PIDFController(P,I,D,F);
        }
    }

    public void setTuningMode(boolean TUNING_MODE) {
        Shooter.TUNING_MODE = TUNING_MODE;
    }





    public int getPosition() {
        return spindexerMotor.getCurrentPosition();
    }

    public Command DirectPower(double power){
        return new InstantCommand(
                ()-> spindexerMotor.setPower(power), this
        );
    }

    public Command NextTarget(){
        return new CommandBase() {
            double target = 0;
            @Override
            public void initialize() {
                target = SpindexerPosition.getNextShootPosition((int)prevtarget);
                spidexerPIDF.setSetpoint(target * 22.755);
                prevtarget = target;

            }

            @Override
            public void execute() {
                spidexerPIDF.getOutput(spindexerMotor.getCurrentPosition(),target);

            }

            @Override
            public void end(boolean interrupted) {
                spindexerMotor.setPower(0);

            }

            @Override
            public boolean isFinished() {
                return Math.abs(spindexerMotor.getCurrentPosition() - target) < 20;
            }
        };

    }

    public void Telemetry(Telemetry telemetry){
        telemetry.addData("Spindexer Position", spindexerMotor.getCurrentPosition());
        telemetry.addData("Spindexer Target", prevtarget);
        telemetry.addData("error",prevtarget - (double)spindexerMotor.getCurrentPosition());
    }

    public Command NextSlot(){
        return new CommandBase() {
            double target = 0;
            @Override
            public void initialize() {
               target = SpindexerPosition.getNextShootPosition((int)prevtarget);
               prevtarget = target;
            }

            @Override
            public void execute() {
                spindexerMotor.setPower(1);
            }

            @Override
            public void end(boolean interrupted) {
                spindexerMotor.setPower(0);

            }

            @Override
            public boolean isFinished() {
                return spindexerMotor.getCurrentPosition() >= target;
            }
        };
    }
}
