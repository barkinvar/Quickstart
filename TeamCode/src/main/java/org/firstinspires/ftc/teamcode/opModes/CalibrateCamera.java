package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision;

@TeleOp(name = "Calibrate Camera", group = "Vision")
public class CalibrateCamera extends OpMode {

    private Vision vision;

    @Override
    public void init() {
        vision = new Vision(telemetry, hardwareMap.get(WebcamName.class, "Arducam"));
        vision.init();
        telemetry.addLine("Vision initialized. Use Dashboard to tune settings.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Apply dashboard-updated camera settings each loop
        vision.updateCameraSettings();
    }
}
