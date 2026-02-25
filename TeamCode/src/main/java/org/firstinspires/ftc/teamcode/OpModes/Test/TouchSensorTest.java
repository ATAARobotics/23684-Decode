package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystem.Touch;
import org.firstinspires.ftc.teamcode.Subsystem.Transfer;


@TeleOp(name = "Touch Sensor Test", group = " Test")
public class TouchSensorTest extends OpMode {
    Spindexer spindexer;

    Intake intake;

    Transfer transfer;
    CommandScheduler scheduler;

   Touch touch;

    private boolean touchSensorTouched = false;
    private boolean aButtonPressed = false;

    @Override
    public void init() {
        scheduler = CommandScheduler.getInstance();
        scheduler.reset();
        spindexer = new Spindexer(hardwareMap);
        transfer = new Transfer(hardwareMap);
        intake = new Intake(hardwareMap);
        touch = new Touch(hardwareMap);

        spindexer.zeroSpindexer();

        scheduler.schedule(touch.ResetCounter());

    }

    @Override
    public void loop() {

        scheduler.run();

        touch.Update(transfer);

//        if(gamepad1.a && !aButtonPressed) {
//            scheduler.schedule(transfer.TransferIn());
//            scheduler.schedule(intake.In());
//            scheduler.schedule(transfer.IntakeDoorOut());
//            aButtonPressed = true;
//        } else if(!gamepad1.b && aButtonPressed) {
//            scheduler.schedule(transfer.TransferStop());
//            scheduler.schedule(intake.Stop());
//            scheduler.schedule(transfer.IntakeDoorStop());
//            aButtonPressed = false;
//        }
//
//        if(intakeTouchSensor.isPressed() && !touchSensorTouched) {
//            scheduler.schedule(new SequentialCommandGroup(
//                    new WaitCommand(300),
//                    spindexer.NextTarget()
//            ));
//            touchSensorTouched = true;
//        } else if(!intakeTouchSensor.isPressed() && touchSensorTouched) {
//            touchSensorTouched = false;
//        }
//
//        telemetry.addData("Touch Sensor Touched", intakeTouchSensor.isPressed());

        touch.Telemetry(telemetry);




    }
}
