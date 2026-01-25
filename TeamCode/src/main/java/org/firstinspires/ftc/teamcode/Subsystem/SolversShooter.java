//package org.firstinspires.ftc.teamcode.Subsystem;
//
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.seattlesolvers.solverslib.command.SubsystemBase;
//import com.seattlesolvers.solverslib.hardware.motors.Motor;
//
//public class SolversShooter extends SubsystemBase {
//    Motor upperShooter;
//    Motor lowerShooter;
//
//    public static double UPPER_P = 0, UPPER_I = 0, UPPER_D = 0;
//    public static double UPPER_KS = 0, UPPER_KV = 0, UPPER_F = 0;
//
//    public static double LOWER_P = 0, LOWER_I = 0, LOWER_D = 0;
//    public static double LOWER_KS = 0, LOWER_KV = 0, LOWER_F = 0;
//
//
//
//    public SolversShooter(HardwareMap hardwareMap){
//        upperShooter = new Motor(hardwareMap, "upperShooter",20,6000);
//        lowerShooter = new Motor(hardwareMap, "lowerShooter",20,6000);
//
//        upperShooter.setRunMode(Motor.RunMode.VelocityControl);
//        lowerShooter.setRunMode(Motor.RunMode.VelocityControl);
//
//    }
//
//    public void setUpperShooter(double P, double I, double D, double KS, double KV){
//        upperShooter.setPositionCoefficient();
//    }
//
//}
