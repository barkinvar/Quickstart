package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.OptionalDouble;

@Config // Enable FTC Dashboard tuning
@TeleOp
public class TeleopRED extends OpMode {
    public Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private TelemetryManager telemetryM;
    private final PIDController alignPID = new PIDController(0.021, 0,0.0005);
    private int scanning = 0;
    private DcMotorSimple intake, feeder;
    private DcMotorEx shooterL, shooterR;
    private final double ticksPerRev = 28.0;
    private Servo led;
    public Vision vision;
    private VoltageSensor batteryVoltageSensor; // Add member for Voltage Sensor

    // --- Default PIDF variables ---
    public static double SHOOTER_P = 3.35;
    public static double SHOOTER_I = 0.0;
    public static double SHOOTER_D = 0.0;
    public static double SHOOTER_F = 10.75;
    public static double NOMINAL_VOLTAGE = 13.0; // Nominal voltage for default F tuning

    // --- Long Shot PIDF variables ---
    public static double LONG_SHOOTER_P = 3.15;
    public static double LONG_SHOOTER_I = 0.0;
    public static double LONG_SHOOTER_D = 0.0;
    public static double LONG_SHOOTER_F = 10.2;
    public static double LONG_NOMINAL_VOLTAGE = 13.0; // Nominal voltage for long shot F tuning

    // --- Alignment Setpoints ---
    public static double DEFAULT_ALIGN_SETPOINT = 0.0; // degrees
    public static double LONG_ALIGN_SETPOINT = 4.0;   // 2 degrees left

    // --- LED Statuses ---
    private static final int LED_STATUS_IDLE = 0;
    private static final int LED_STATUS_AIMING = 1;
    private static final int LED_STATUS_AT_RPM = 2;


    @Override
    public void init() {
        // Combine standard telemetry with FTC Dashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Get the battery voltage sensor
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();


        vision = new Vision(telemetry, hardwareMap.get(WebcamName.class, "Arducam"));
        follower = Constants.createFollower(hardwareMap);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        intake = hardwareMap.get( DcMotorSimple.class, "intake");
        feeder = hardwareMap.get( DcMotorSimple.class, "feeder");
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose.plus(new Pose(0.000, 0.000, Math.toRadians(90))));


        shooterL = hardwareMap.get( DcMotorEx.class, "shooterL");
        shooterR = hardwareMap.get( DcMotorEx.class, "shooterR");

        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        vision.init();

        led = hardwareMap.get(Servo.class, "led");
        setLedStatus(LED_STATUS_IDLE); // Set to idle on init
        alignPID.setTolerance(1.0);
        alignPID.setSetPoint(DEFAULT_ALIGN_SETPOINT); // Set default align
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        vision.updateCameraSettings();
    }


    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();

        // Get distance once per loop
        double currentDistance = getDistance();

        // --- Shooter Logic ---
        boolean useLongShotPID = false;
        double alignSetpoint = DEFAULT_ALIGN_SETPOINT;
        double targetRPM = 0.0;

        boolean isShooting = gamepad1.b;

        if (isShooting) {
            if (currentDistance > 3.25) {
                // This is the "Long Shot"
                targetRPM = 3400.0;
                useLongShotPID = true; // Use the long shot PID values
                alignSetpoint = LONG_ALIGN_SETPOINT; // Use the long shot alignment
            } else {
                // This is the interpolation shot
                alignSetpoint = DEFAULT_ALIGN_SETPOINT; // Use default align
                if (currentDistance > 1.0) { // Check for valid distance
                    double minDistance = 1.29;
                    double maxDistance = 2.55;
                    double minRPM = 2750.0;
                    double maxRPM = 2970.0;

                    // Calculate the slope (change in RPM per unit of distance)
                    double slope = (maxRPM - minRPM) / (maxDistance - minDistance);
                    // Linear interpolation: RPM = minRPM + (change in distance) * slope
                    double interpolatedRPM = minRPM + (currentDistance - minDistance) * slope;

                    // Clamp the RPM to the defined min/max range
                    targetRPM = Math.max(minRPM, Math.min(maxRPM, interpolatedRPM));
                } else {
                    targetRPM = 2000.0; // Fallback if no tag
                }
            }

            // --- Set Subsystems ---
            setShooterRPM(targetRPM, useLongShotPID);
            boolean atRPM = isShooterAtRPM(targetRPM);

            if (atRPM) {
                setLedStatus(LED_STATUS_AT_RPM);
                runIntake(1.0);
                runFeeder(-1.0);
            } else {
                setLedStatus(LED_STATUS_AIMING);
                runFeeder(0.0); // Stop feeder if not at RPM
                if (gamepad1.right_bumper) {
                    runIntake(1.0);
                } else {
                    runIntake(0.0);
                }
            }

            // --- Handle Driving ---
            handleAutoAlignDrive(alignSetpoint, -gamepad1.left_stick_y, -gamepad1.left_stick_x);

        } else {
            // --- Not in Shooting Mode ---
            scanning = 0;
            stopShooter();
            setLedStatus(LED_STATUS_IDLE);

            // Handle Intake/Feeder
            runFeeder(0.0);
            if (gamepad1.right_bumper) {
                runIntake(1.0);
            } else {
                runIntake(0.0);
            }

            // --- Handle Driving ---
            handleManualDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x / 1.5);
        }

        // --- Telemetry ---
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetry.addData("tagDistance", currentDistance); // Use the variable
        telemetry.addData("RPM-R", (shooterR.getVelocity() / ticksPerRev) * 60.0);
        telemetry.addData("RPM-L", (shooterL.getVelocity() / ticksPerRev) * 60.0);

        // Add tunable values and voltage telemetry
        double currentVoltage = batteryVoltageSensor.getVoltage();
        telemetry.addData("Battery Voltage", currentVoltage);
        telemetry.addData("Shooter P", useLongShotPID ? LONG_SHOOTER_P : SHOOTER_P);
        telemetry.addData("Shooter I", useLongShotPID ? LONG_SHOOTER_I : SHOOTER_I);
        telemetry.addData("Shooter D", useLongShotPID ? LONG_SHOOTER_D : SHOOTER_D);
        telemetry.addData("Shooter F (Base)", useLongShotPID ? LONG_SHOOTER_F : SHOOTER_F);
        telemetry.addData("Align Setpoint", alignSetpoint);

        // Avoid division by zero if voltage is very low
        if (currentVoltage > 0.1) {
            double compensatedF = (useLongShotPID ? LONG_SHOOTER_F : SHOOTER_F) * ((useLongShotPID ? LONG_NOMINAL_VOLTAGE : NOMINAL_VOLTAGE) / currentVoltage);
            telemetry.addData("Compensated F", compensatedF);
        } else {
            telemetry.addData("Compensated F", "N/A (Low Voltage)");
        }

        telemetry.update();
    }

    //================================================================================
    // PUBLIC SUBSYSTEM METHODS (for Auto)
    //================================================================================

    /**
     * Sets the shooter motors to a target RPM, applying the correct PIDF values
     * and compensating for battery voltage.
     * @param RPM The target RPM.
     * @param useLongShotPID true to use LONG_SHOOTER values, false to use default SHOOTER values.
     */
    public void setShooterRPM(double RPM, boolean useLongShotPID) {
        double p, i, d, f, nominalV;

        if (useLongShotPID) {
            p = LONG_SHOOTER_P;
            i = LONG_SHOOTER_I;
            d = LONG_SHOOTER_D;
            f = LONG_SHOOTER_F;
            nominalV = LONG_NOMINAL_VOLTAGE;
        } else {
            p = SHOOTER_P;
            i = SHOOTER_I;
            d = SHOOTER_D;
            f = SHOOTER_F;
            nominalV = NOMINAL_VOLTAGE;
        }

        double currentVoltage = batteryVoltageSensor.getVoltage();
        // Avoid division by zero or nonsensical values
        if (currentVoltage <= 0.1) {
            currentVoltage = nominalV;
        }

        double compensatedF = f * (nominalV / currentVoltage);

        shooterL.setVelocityPIDFCoefficients(p, i, d, compensatedF);
        shooterR.setVelocityPIDFCoefficients(p, i, d, compensatedF);

        double velocity = (RPM / 60.0) * ticksPerRev;
        shooterR.setVelocity(-velocity);
        shooterL.setVelocity(velocity);
    }

    /**
     * Stops the shooter motors.
     */
    public void stopShooter() {
        setShooterRPM(0, false);
    }

    /**
     * Checks if the shooter is within a tolerance of the target RPM.
     * @param setRPM The target RPM.
     * @return true if the shooter is at the target RPM.
     */
    public boolean isShooterAtRPM(double setRPM) {
        double rpm = (shooterR.getVelocity() / ticksPerRev) * 60.0;
        boolean atRPM = Math.abs(setRPM - rpm) < 200.0;
        telemetryM.addData("isAtRPM", atRPM);
        return atRPM;
    }

    public boolean isShooterAtRPMAuto(double setRPM) {
        double rpm = (shooterR.getVelocity() / ticksPerRev) * 60.0;
        boolean atRPM = Math.abs(setRPM - rpm) < 300.0;
        telemetryM.addData("isAtRPM", atRPM);
        return atRPM;
    }


    /**
     * Sets the power of the intake motor.
     * @param power Power from -1.0 to 1.0.
     */
    public void runIntake(double power) {
        intake.setPower(power);
    }

    /**
     * Sets the power of the feeder motor.
     * @param power Power from -1.0 to 1.0.
     */
    public void runFeeder(double power) {
        feeder.setPower(power);
    }

    /**
     * Sets the LED servo to a specific status.
     * 0 = Idle, 1 = Aiming (not at RPM), 2 = At RPM (ready to fire).
     * @param status The status code (0, 1, or 2).
     */
    public void setLedStatus(int status) {
        if (status == LED_STATUS_AT_RPM) {
            led.setPosition(0.480); // At RPM
        } else if (status == LED_STATUS_AIMING) {
            led.setPosition(0.611); // Aiming, not at RPM
        } else {
            led.setPosition(0.28); // Idle
        }
    }

    /**
     * Gets the current distance from the vision target.
     * @return Distance in meters, or 0.0 if not found.
     */
    public double getDistance() {
        return vision.getDistance(24);
    }

    //================================================================================
    // PRIVATE TELEOP-ONLY HELPER METHODS
    //================================================================================

    /**
     * Handles manual, field-centric driving.
     */
    private void handleManualDrive(double ySpeed, double xSpeed, double rotation) {
        follower.setTeleOpDrive(
                ySpeed,
                xSpeed,
                rotation,
                false // Field Centric
        );
    }

    /**
     * Handles driving while automatically aligning to a vision target.
     * @param targetYaw The desired yaw setpoint (in degrees).
     * @param ySpeed Driver's forward/backward input.
     * @param xSpeed Driver's strafe input.
     * @return true if the robot is at the setpoint.
     */
    private boolean handleAutoAlignDrive(double targetYaw, double ySpeed, double xSpeed) {
        // Update the PID setpoint every time we align
        alignPID.setSetPoint(targetYaw);

        OptionalDouble visionYaw = vision.getYaw(24);
        boolean atSetpoint = false;
        double rotation;

        if(visionYaw.isPresent()){
            double yaw = visionYaw.getAsDouble();
            rotation = -alignPID.calculate(yaw); // PID calculates rotation
            atSetpoint = alignPID.atSetPoint();
            scanning = 0;
        }
        else if(scanning == 0){
            // Start scanning if we lose the tag
            scanning = (follower.getHeading() > Math.toRadians(-35.0)) ? 1 : -1;
            rotation = (scanning == 1) ? -0.35 : 0.35;
        }
        else {
            // Continue scanning
            rotation = (scanning == 1) ? -0.35 : 0.35;
        }

        follower.setTeleOpDrive(
                ySpeed,
                xSpeed,
                rotation,
                false // Field Centric
        );

        return atSetpoint;
    }

    /**
     * This method is no longer used, but kept as an example.
     */
    public void runShooterPercentage(double percent) {
        shooterL.setPower(percent);
        shooterR.setPower(-percent);
    }
}