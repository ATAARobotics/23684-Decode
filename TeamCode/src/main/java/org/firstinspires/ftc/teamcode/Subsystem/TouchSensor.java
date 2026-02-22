package org.firstinspires.ftc.teamcode.Subsystem;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

public class TouchSensor extends SubsystemBase {

    private com.qualcomm.robotcore.hardware.TouchSensor intakeTouchSensorRight;
    private com.qualcomm.robotcore.hardware.TouchSensor intakeTouchSensorLeft;

    private com.qualcomm.robotcore.hardware.TouchSensor transferMagSwitchRight;
    private com.qualcomm.robotcore.hardware.TouchSensor transferMagSwitchLeft;

    boolean intakePressed = false;
    boolean transferPressed = false;

    boolean finalizedCycle = false;


    public int artifactCollectCount = 0;

    public int arifactShotCount = 0;

    public TouchSensor(HardwareMap hardwareMap){
        intakeTouchSensorRight = hardwareMap.get(com.qualcomm.robotcore.hardware.TouchSensor.class,"intakeTouchSensorRight");
        intakeTouchSensorLeft = hardwareMap.get(com.qualcomm.robotcore.hardware.TouchSensor.class, "intakeTouchSensorLeft");

        transferMagSwitchRight = hardwareMap.get(com.qualcomm.robotcore.hardware.TouchSensor.class, "transferMagSwitchRight");
        transferMagSwitchLeft = hardwareMap.get(com.qualcomm.robotcore.hardware.TouchSensor.class, "transferMagSwitchLeft");

    }

    public Command ResetCounter(){
        return new InstantCommand(
                () -> {

                        artifactCollectCount = 0;
                        arifactShotCount = 0;
                        finalizedCycle = false;

                }, this
        );

    }

    public boolean ArtifactInIntake(){
        return intakeTouchSensorRight.isPressed() || intakeTouchSensorLeft.isPressed();

    }

    public boolean ArtifactInTransfer() {
        return !transferMagSwitchRight.isPressed() || !transferMagSwitchLeft.isPressed();
    }

    public Command ShootAllArtifacts(){
        return new ParallelCommandGroup(
                new InstantCommand(()-> finalizedCycle = true),
                new WaitUntilCommand(()-> arifactShotCount == artifactCollectCount));
    }

    public void Update(Transfer transfer){

        if (ArtifactInIntake() && !intakePressed && !finalizedCycle) {
            artifactCollectCount += 1;
            intakePressed = true;
        }else if (!ArtifactInIntake() && intakePressed) {
            intakePressed = false;
        }

        if (ArtifactInTransfer() && !transferPressed && transfer.reachedLowerTarget && transfer.reachedUpperTarget) {
            arifactShotCount += 1;
            transferPressed = true;
        }else if (!ArtifactInTransfer() && transferPressed){
            transferPressed = false;
        }


   }

   public void Telemetry(TelemetryManager TelemetryManager){
        TelemetryManager.addData("Artifact Count", artifactCollectCount);
        TelemetryManager.addData("Artifact Shot Count", arifactShotCount);

   }







    }
