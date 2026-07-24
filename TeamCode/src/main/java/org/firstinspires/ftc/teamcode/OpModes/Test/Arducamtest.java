package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystem.ArduCam;


@TeleOp(name = "Arducam Test", group = "Test")
public class Arducamtest extends OpMode {
    ArduCam arduCam;
    @Override
    public void init() {
        arduCam = new ArduCam(hardwareMap);
    }

    @Override
    public void loop() {
        arduCam.ArduCamTelemetry(telemetry);

    }
}
