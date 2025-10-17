package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.concurrent.TimeUnit;

public class Vision {
    private final Telemetry telemetry;
    private VisionPortal visionPortal;
    private final WebcamName webcam;
    private AprilTagProcessor processor;

    private final double CAMERA_HEIGHT = 0.34;
    private final double TAG_HEIGHT = 29.25 * 0.0254;
    private final double CAMERA_TILT = 25.0;

    final double fx = 913.538;
    final double fy = 913.538;
    final double cx = 631.279;
    final double cy = 378.085;

    public Vision(Telemetry telemetry, WebcamName webcam) {
        this.telemetry = telemetry;
        this.webcam = webcam;
    }

    public void init() {
        telemetry.addLine("Initializing camera...");
        telemetry.update();

        processor = new AprilTagProcessor.Builder().build();


        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(processor)
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCameraResolution(new Size(
                        640,
                        480
                ))
                .build();

        FtcDashboard.getInstance().startCameraStream(visionPortal, 10);

        telemetry.addLine("Camera initialized.");
        telemetry.update();
    }

    public void updateCameraSettings() {
        if (visionPortal == null) return;

        visionPortal.getCameraControl(ExposureControl.class).setMode(ExposureControl.Mode.Manual);
        visionPortal.getCameraControl(ExposureControl.class).setMode(ExposureControl.Mode.Manual);
        visionPortal.getCameraControl(ExposureControl.class).setMode(ExposureControl.Mode.Manual);
        visionPortal.getCameraControl(ExposureControl.class).setExposure(VisionConstants.EXPOSURE_MS, TimeUnit.MICROSECONDS);

        visionPortal.getCameraControl(GainControl.class).setGain(VisionConstants.GAIN);

        WhiteBalanceControl wbControl = visionPortal.getCameraControl(WhiteBalanceControl.class);
        wbControl.setMode(WhiteBalanceControl.Mode.MANUAL);
        wbControl.setMode(WhiteBalanceControl.Mode.MANUAL);
        wbControl.setMode(WhiteBalanceControl.Mode.MANUAL);

        wbControl.setWhiteBalanceTemperature(VisionConstants.WB);
    }

    public List<AprilTagDetection> getDetections() {
        return processor.getDetections();
    }

    public Optional<AprilTagDetection> getDetectionById(int id) {
        List<AprilTagDetection> detections = getDetections();
        if (detections != null) {
            for (AprilTagDetection detection : detections) {
                if (detection.id == id) {
                    return Optional.of(detection);
                }
            }
        }
        return Optional.empty();
    }

    public OptionalDouble getYaw(int id) {
        Optional<AprilTagDetection> detection = getDetectionById(id);
        return detection.map(aprilTagDetection -> OptionalDouble.of(aprilTagDetection.ftcPose.bearing))
                .orElseGet(OptionalDouble::empty);
    }

    public double getDistance(int id) {
        Optional<AprilTagDetection> detection = getDetectionById(id);
        if(!detection.isPresent()) {return  0;}

        double detectionAngle = detection.get().ftcPose.elevation;

        telemetry.addData("detection Angle", detectionAngle);

        return (TAG_HEIGHT - CAMERA_HEIGHT) / Math.tan(Math.toRadians(CAMERA_TILT + detectionAngle));
    }

    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}


@Config
class VisionConstants {
    // Default values (will appear live on FTC Dashboard)
    public static long EXPOSURE_MS = 2250;
    public static int GAIN = 15;
    public static int WB = 1;

}