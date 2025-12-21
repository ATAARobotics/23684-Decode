package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Utilities.SpindexerPosition;

public class ShootThreeAction {
    private Spindexer spindexer;
    private Shooter shooter;
    private Transfer transfer;
    private Intake intake;
    private int shooterTarget1;
    private int shooterTarget2;
    private int shooterTarget3;

    public ShootThreeAction(){
        spindexer = Spindexer.getInstance();
        shooter = Shooter.getInstance();
        transfer = Transfer.getInstance();
        intake = Intake.getInstance();

        shooterTarget1 = SpindexerPosition.getNextShootPosition(0);
        shooterTarget2 = SpindexerPosition.getNextShootPosition(shooterTarget1);
        shooterTarget3 = SpindexerPosition.getNextShootPosition(shooterTarget2);

    }

    public Action ShootThreeArtifacts(){
        return new SequentialAction(
                transfer.intakeDoorForward(),
                intake.slow(),
                spindexer.setTarget(shooterTarget1),
                shooter.runAndWait(Shooter.AUDIENCE_RPM, Shooter.AUDIENCE_RPM),
                // TODO: Extract into a helper function later
                new Action() {
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
                },
                spindexer.setTarget(shooterTarget2),
                shooter.runAndWait(Shooter.AUDIENCE_RPM, Shooter.AUDIENCE_RPM),
                new Action() {
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
                },
                spindexer.setTarget(shooterTarget3),
                shooter.runAndWait(Shooter.AUDIENCE_RPM, Shooter.AUDIENCE_RPM),
                new Action() {
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
                },
                shooter.stop()
        );
    }
}
