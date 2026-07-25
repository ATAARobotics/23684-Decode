package org.firstinspires.ftc.teamcode.Subsystem;

import android.util.Size;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.FilteredPIDFController;
import com.pedropathing.control.KalmanFilter;
import com.pedropathing.control.KalmanFilterParameters;
import com.pedropathing.control.LowPassFilter;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Utils.Drawing;
import org.firstinspires.ftc.teamcode.Utils.Team;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class ArduCam {

    private Position cameraPosition = new Position(DistanceUnit.MM,
            0, -16.5, 390, 0); //TODO: Check for actual position
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            180, -75, 0, 0);

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    protected TelemetryManager.TelemetryWrapper panelsTelemetry;

    KalmanFilter xLowPassFilter;
    KalmanFilter yLowPassFilter;




    public ArduCam(HardwareMap hardwareMap){

        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .setLensIntrinsics(908.758f, 908.758f, 696.345f, 376.979f )
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        aprilTag.setDecimation(2);

        builder
                .setCamera(hardwareMap.get(WebcamName.class, "0xC45"))
                .setCameraResolution(new Size(1280, 800 ))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag);


        visionPortal = builder.build();
        panelsTelemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();

        if(visionPortal != null && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {

            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
            }
            exposureControl.setExposure(1, TimeUnit.MILLISECONDS);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(260);
        }

        xLowPassFilter = new KalmanFilter(new KalmanFilterParameters(60,70));
        yLowPassFilter = new KalmanFilter(new KalmanFilterParameters(60,70));

    }

    public void ArduCamTelemetry(Telemetry telemetry) {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        telemetry.addData("AprilTags Detected?", AnyGoalsFound());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                // Only use tags that don't have Obelisk in them
                if (!detection.metadata.name.contains("Obelisk")) {
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getPosition().z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));

                    telemetry.addData("blue bearing", angleFrom(Team.BLUE));
                    telemetry.addData("red bearing", angleFrom(Team.RED));


                    //Pose vispose = new Pose(detection.robotPose.getPosition().x, detection.robotPose.getPosition().y, detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS));

                    Drawing.drawRobot(VisualPose());
                    Drawing.sendPacket();

                    telemetry.addData("pose guess", VisualPose().toString());

                }

                // code to try to convert it to somthing pedropathing can use


                // Add "key" information to telemetry


            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
           panelsTelemetry.update();
        }
    }

    public boolean AnyGoalsFound() {


        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                // Only use tags that don't have Obelisk in them
                if (detection.metadata.id == 20 || detection.metadata.id == 24) return true;
            }else return false;
        }
        return false;
    }

    public boolean GoalsFound(Team team) {


        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                // Only use tags that don't have Obelisk in them
                if (team == Team.BLUE) {
                    if (detection.metadata.id == 20 ) return true;
                } else if (team == Team.RED) {
                    if (detection.metadata.id == 24) return true;
                }
            }else return false;
        }
        return false;
    }

    public double angleFrom(Team team) {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                // Only use tags that don't have Obelisk in them
                if (team == Team.BLUE) {
                    if (detection.metadata.id == 20) {
                       return detection.ftcPose.bearing;
                    }
                }else if (team == Team.RED) {
                    if (detection.metadata.id == 24) {
                        return detection.ftcPose.bearing;
                    }
                }


            } else return 0;

        }
        return 0;
    }

    public Pose VisualPose() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.metadata.id == 20 || detection.metadata.id == 24) {
//                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
//                            detection.robotPose.getPosition().x,
//                            detection.robotPose.getPosition().y,
//                            detection.robotPose.getPosition().z));
//                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
//                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
//                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
//                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));

                    xLowPassFilter.update(detection.robotPose.getPosition().x,0.99);
                    yLowPassFilter.update(detection.robotPose.getPosition().y,0.99);

                    Pose vispose = PoseConverter.pose2DToPose(new Pose2D(
                            DistanceUnit.INCH,
                            xLowPassFilter.getState(),
                            yLowPassFilter.getState(),
                            AngleUnit.DEGREES,
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)
                    ), InvertedFTCCoordinates.INSTANCE);

                    //Pose vispose = new Pose(detection.robotPose.getPosition().x, detection.robotPose.getPosition().y, detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS));

                    return vispose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);

                }
            } else{
                xLowPassFilter.update(0,0.8);
                yLowPassFilter.update(0,0.8);
                return new Pose(0,0,0);}
        }
        return new Pose(0,0,0);
    }

}
