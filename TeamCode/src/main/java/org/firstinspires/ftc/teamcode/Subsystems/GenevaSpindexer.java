package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class GenevaSpindexer {

    DcMotor frontLeft;
    CRServo spindexerLeft;

    public GenevaSpindexer(HardwareMap hardwareMap){
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        spindexerLeft = hardwareMap.get(CRServo.class, "spindexerLeft");

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public class NextSlot implements Action {
        double target = 0;
        boolean checked = false;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!checked) {
                target = frontLeft.getCurrentPosition() + 120;
                checked = true;
            }else{
                spindexerLeft.setPower(1);
            }

            return frontLeft.getCurrentPosition() >= target;
        }
    }

    public Action NextSlot(){
        return new NextSlot();
    }

    public Action Power(double power){
        return new InstantAction(
                () -> {
                    frontLeft.setPower(power);
                }
        );
    }
}
