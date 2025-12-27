package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.sql.Time;

public class ShootPatterns {
    private Spindexer spindexer;
    private Shooter shooter;
    private Transfer transfer;
    private Intake intake;

    public ShootPatterns(HardwareMap hardwareMap){
        Shooter.initialize(hardwareMap);
        shooter = Shooter.getInstance();

        Transfer.initialize(hardwareMap);
        transfer = Transfer.getInstance();

        Intake.initialize(hardwareMap);
        intake = Intake.getInstance();

        Spindexer.initialize(hardwareMap);
        spindexer = Spindexer.getInstance();
    }

    public Action ShootWithTime(){
        return new Action() {
            long startTime = -1;

            @Override
            public boolean run(TelemetryPacket telemetryPacket) {
                // Initialize startTime on the first run
                if (startTime == -1) {
                    startTime = System.nanoTime();
                }

                // Calculate how much time has passed
                long elapsedTime = System.nanoTime() - startTime;

                // Check if less than 1 second (1,000,000,000 nanoseconds) has passed
                if (elapsedTime < 1_000_000_000L) {
                    shooter.run(Shooter.AUDIENCE_RPM).run(new TelemetryPacket());
                    return true; // Continue running this action
                } else {
                    return false; // Action is done
                }
            }
        };
    }

    public Action Wait(long secs){
        return new Action() {
            long time = -1;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (time == -1) {
                    time = System.nanoTime();
                }
                long elapsedTime = System.nanoTime() - time;
                return elapsedTime <= secs;
            }
        };
    }
    public Action ShootPatern( int patternNumber)  {

           if(patternNumber == 123) {
                return new SequentialAction(
                        new ParallelAction(
                        intake.slow(),
                        //shooter.run(Shooter.AUDIENCE_RPM, Shooter.AUDIENCE_RPM),
                        transfer.transferIn(),
                        spindexer.setDirectPower(1)
                                ),
                       new RaceAction(
                        shooter.MindlessRun(Shooter.AUDIENCE_RPM,Shooter.AUDIENCE_RPM),
                         new SleepAction(2.8)
                       ),
                        new ParallelAction(
                        intake.stop(),
                        spindexer.setDirectPower(0),
                        shooter.stop()
                        )

                );
            } else if (patternNumber == 312) {
               return new SequentialAction(
                       transfer.intakeDoorBackward(),
                       shooter.runAndWait(Shooter.AUDIENCE_RPM, Shooter.AUDIENCE_RPM),
                       spindexer.TwoSlots(),
                       transfer.intakeDoorForward(),
                       shooter.WaitForShoot(),
                       spindexer.setDirectPower(1),
                       shooter.WaitForShoot(),
                       shooter.WaitForShoot()
               );
           } else if (patternNumber == 132) {
               return new SequentialAction(

                       shooter.run(Shooter.AUDIENCE_RPM, Shooter.AUDIENCE_RPM),
                       transfer.transferOut(),
                       shooter.WaitForShoot(),
                       transfer.transferIn(),
                       spindexer.TwoSlots(),
                       transfer.transferOut(),
                       spindexer.setDirectPower(1),
                       shooter.WaitForShoot(),
                       shooter.WaitForShoot()
               );
           } else if (patternNumber == 321) {
               return new SequentialAction(
                           shooter.runAndWait(Shooter.AUDIENCE_RPM, Shooter.AUDIENCE_RPM),
                           transfer.intakeDoorBackward(),
                           spindexer.TwoSlots(),
                           transfer.intakeDoorForward(),
                           shooter.WaitForShoot(),
                           spindexer.setDirectPower(1),
                           shooter.WaitForShoot(),
                           shooter.WaitForShoot()

                   );
           } else if (patternNumber == 213) {
               return new SequentialAction(
                       shooter.runAndWait(Shooter.AUDIENCE_RPM, Shooter.AUDIENCE_RPM),
                       transfer.intakeDoorBackward(),
                       spindexer.NextSlot(),
                       transfer.intakeDoorForward(),
                       shooter.WaitForShoot(),
                       transfer.intakeDoorBackward(),
                       spindexer.TwoSlots(),
                       transfer.intakeDoorForward(),
                       shooter.WaitForShoot(),
                       spindexer.setDirectPower(1),
                       shooter.WaitForShoot()
               );
           }
        return new InstantAction(() -> {
            throw new IllegalStateException("Bro, that pattern isn't real");
        }
        );
    }
}
