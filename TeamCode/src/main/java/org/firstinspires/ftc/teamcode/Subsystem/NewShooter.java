package org.firstinspires.ftc.teamcode.Subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;


@Configurable
public class NewShooter extends SubsystemBase {

    public DcMotorEx upperShooter;
    public DcMotorEx lowerShooter;

    public static double UpperP = 16, UpperF = 14;
    public static double LowerP = 15 ,LowerF = 16;

    public static double AUDIENCE_TPR = 1050;

    private double upperSpeed, lowerSpeed;


    public NewShooter(HardwareMap hardwareMap){
        upperShooter = hardwareMap.get(DcMotorEx.class, "upperShooter");
        lowerShooter = hardwareMap.get(DcMotorEx.class, "lowerShooter");

        PIDFCoefficients pidfCoefficientsUpper = new PIDFCoefficients(UpperP, 0, 0, UpperF);
        PIDFCoefficients pidfCoefficientsLower = new PIDFCoefficients(LowerP, 0, 0, LowerF);

        upperShooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficientsUpper);
        lowerShooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficientsLower);

    }

    @Override
    public void periodic(){

        PIDFCoefficients pidfCoefficientsUpper = new PIDFCoefficients(UpperP, 0, 0, UpperF);
        PIDFCoefficients pidfCoefficientsLower = new PIDFCoefficients(LowerP, 0, 0, LowerF);

        upperShooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficientsUpper);
        lowerShooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficientsLower);

        upperShooter.setVelocity(upperSpeed);
        lowerShooter.setVelocity(lowerSpeed);
    }

    public boolean isAtTarget(){
        boolean upper = (Math.abs(upperSpeed - upperShooter.getVelocity()) < 50) && upperShooter.getVelocity() > 500;
        boolean lower = (Math.abs(lowerSpeed - lowerShooter.getVelocity()) < 50) & lowerShooter.getVelocity() > 500;
        return upper && lower;
    }

    public Command setTarget(double upperShooter, double lowerShooter){
        return new InstantCommand(
                ()->{
                    upperSpeed = upperShooter;
                    lowerSpeed = lowerShooter;
                },this
        );
    }

    public Command WaitForTarget(){
        return new WaitUntilCommand(this::isAtTarget);
    }
}
