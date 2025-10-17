package org.firstinspires.ftc.teamcode.opModes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.DrawingPublic;

import java.util.OptionalDouble;

@TeleOp
public class ExampleTeleOp extends OpMode {
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private TelemetryManager telemetryM;
    private final PIDController alignPID = new PIDController(0.012, 0,0.0005);
    private int scanning = 0;
    private Vision vision;
    @Override
    public void init() {
        vision = new Vision(telemetry, hardwareMap.get(WebcamName.class, "Arducam"));
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();


        vision.init();

        alignPID.setTolerance(1.0);
        alignPID.setSetPoint(0.0);
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
        vision.updateCameraSettings();

    }


    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();

        DrawingPublic.drawDebug(follower);

        if(gamepad1.a) {
            alignToTarget();
        }
        else {
            scanning = 0;
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x / 1.5,
                    false // Field Centric
            );
        }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetry.addData("tagDistance", vision.getDistance(20));
        telemetry.update();
    }

    public boolean alignToTarget() {
        OptionalDouble visionYaw = vision.getYaw(20);

        boolean atSetpoint = false;
        double rotation = -gamepad1.right_stick_x / 1.5;

        if(visionYaw.isPresent()){
            double yaw = visionYaw.getAsDouble();
            rotation = -alignPID.calculate(yaw);

            atSetpoint = alignPID.atSetPoint();

            scanning = 0;
        }
        else if(scanning == 0){
            if (follower.getHeading() > 0) {
                scanning = 1;
            }
            else {
                scanning = -1;
            }
        }
        else {
            rotation = scanning == 1 ? -0.35 : 0.35;
        }

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                rotation,
                false // Field Centric
        );

        return atSetpoint;
    }
}
