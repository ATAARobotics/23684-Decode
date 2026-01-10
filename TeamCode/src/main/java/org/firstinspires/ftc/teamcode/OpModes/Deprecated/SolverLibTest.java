//package org.firstinspires.ftc.teamcode.OpModes.Deprecated;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.seattlesolvers.solverslib.command.CommandScheduler;
//import com.seattlesolvers.solverslib.command.InstantCommand;
//import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
//import com.seattlesolvers.solverslib.command.WaitCommand;
//
//@Autonomous
//public class SolverLibTest extends OpMode {
//
//    // SolverLib is a fork by the seatle slovers  of ftclib
//    // SloverLib alows us to Commands with pedropathing and
//    // the other option is to use a switch statement
//
//    CommandScheduler scheduler;
//
//
//    Servo oven;
//    @Override
//    public void init() {
//
//        scheduler = CommandScheduler.getInstance();
//        oven = hardwareMap.get(Servo.class, "billy");
//
//    }
//
//    @Override
//    public void start() {
//
//        scheduler.schedule(
//                new SequentialCommandGroup(
//                        new InstantCommand(() -> oven.setPosition(0)),
//                        new WaitCommand(2800),
//                        new InstantCommand(()-> oven.setPosition(1)),
//                        new WaitCommand(2800),
//                        new InstantCommand(()-> telemetry.addLine("We did it patrick!"))
//                )
//        );
//
//    }
//
//
//    @Override
//    public void loop() {
//        telemetry.update();
//        scheduler.run();
//
//    }
//}
