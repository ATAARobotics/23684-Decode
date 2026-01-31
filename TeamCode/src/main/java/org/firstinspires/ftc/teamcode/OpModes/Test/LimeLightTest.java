package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.Limelight;


@TeleOp
public class LimeLightTest extends OpMode {

    Limelight limelight;

    TelemetryManager telemetryManager;

    Follower follower;

    @Override
    public void init() {
        limelight = new Limelight(hardwareMap);
        telemetryManager =  PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(new Pose(72,72,0));

    }
    @Override
    public void start(){
        limelight.start();
    }

    @Override
    public void loop() {
        limelight.setHeading(follower.getHeading());
        limelight.Telemetry(telemetry);

    }
}
