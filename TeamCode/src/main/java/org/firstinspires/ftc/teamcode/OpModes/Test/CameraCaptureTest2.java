package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.teamcode.Utils.AprilTagPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;

@TeleOp(name="cam", group="Test")
public class CameraCaptureTest2 extends LinearOpMode{

    boolean A;
    boolean aFlag= false;

    public AprilTagPipeline cameraPipeline = new AprilTagPipeline(20);

    @Override
    public void runOpMode() {

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvWebcam frontCamera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);

        frontCamera.setPipeline( cameraPipeline );

        frontCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                frontCamera.startStreaming(1920, 1200, OpenCvCameraRotation.UPRIGHT);
                frontCamera.getWhiteBalanceControl().setMode(WhiteBalanceControl.Mode.MANUAL);
                frontCamera.getWhiteBalanceControl().setWhiteBalanceTemperature(3000);// 3000
                frontCamera.getExposureControl().setMode(ExposureControl.Mode.Manual);
                frontCamera.getExposureControl().setExposure(5, TimeUnit.MILLISECONDS); // 20
                frontCamera.getGainControl().setGain(90); // 20
            }
            @Override
            public void onError(int errorCode) {}
        });

        while (opModeInInit()) {

            //
        }

        waitForStart();
        while (opModeIsActive()) {
//            telemetry.addLine("I don't do anything!");

            if (cameraPipeline.has_result() && cameraPipeline.getLatestDetections() != null){
                for (int i = 0; i < cameraPipeline.getLatestDetections().size(); i++) {
                    telemetry.addLine("Tag ID: " + cameraPipeline.getLatestDetections().get(i).id);
                    telemetry.addLine("Tag pose: " + cameraPipeline.getLatestDetections().get(i).pose);
                }
            }
            telemetry.addData("fps",frontCamera.getFps());
            telemetry.update();

        }
    }
}