package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystem.Spindexer;



@TeleOp
public class DistanceSensorTest extends OpMode {

    DistanceSensor distanceSensor;

    Spindexer spindexer;

    CommandScheduler scheduler;

    boolean ArtifactFound = false;


    @Override
    public void init() {

        distanceSensor = hardwareMap.get(DistanceSensor.class, "intakeDistanceSensor");

        spindexer = new Spindexer(hardwareMap);

        scheduler = CommandScheduler.getInstance();


    }

    public void loop() {
        scheduler.run();

        boolean TriggeredDistance = distanceSensor.getDistance(DistanceUnit.CM) < 20;

        if(TriggeredDistance && !ArtifactFound){
            scheduler.schedule(new SequentialCommandGroup(new WaitCommand(100),spindexer.NextTarget()));
            ArtifactFound = true;
        }if(!TriggeredDistance && ArtifactFound){
            ArtifactFound = false;
        }

        telemetry.addData("Distance", distanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Artifact Found", ArtifactFound);
        telemetry.update();
    }


}
