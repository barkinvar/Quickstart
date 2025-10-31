package org.firstinspires.ftc.teamcode.opModes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDController;

import java.util.OptionalDouble;

// 1. EXTEND ExampleTeleOp2 to inherit all hardware and public methods
@Autonomous(name = "Autonomous Red")
@Configurable // Panels
public class AutoRED extends TeleopRED {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private ElapsedTime timer = new ElapsedTime(); // Timer for state machine actions

    // PID controller for autonomous alignment
    private PIDController autoAlignPID = new PIDController(0.03, 0, 0.0005);

    @Override
    public void init() {
        // 2. Call super.init() to initialize the follower, motors, vision, etc.
        super.autoInit();


        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // 3. Set the starting pose. This MUST match the start of your first path.
        // Your Path1 starts at (56, 8) with a heading of 90 deg.
        follower.setPose(new Pose(88.000, 8.000, Math.toRadians(90)));

        // 4. Build paths using the *follower* variable from ExampleTeleOp2
        paths = new Paths(follower);

        // Configure the alignment PID
        autoAlignPID.setTolerance(1.0); // 1 degree tolerance

        // Reset the state machine
        pathState = 0;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        telemetry.update();
    }
    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(88.000, 8.000), new Pose(88.000, 15.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(68))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(88.000, 15.000),
                                    new Pose(95.707, 38.634),
                                    new Pose(133.683, 35.122)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(133.683, 35.122),
                                    new Pose(97.463, 44.122),
                                    new Pose(82.098, 15.146)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(82.098, 15.146),
                                    new Pose(93.293, 64.098),
                                    new Pose(134.780, 58.829)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(134.780, 58.829),
                                    new Pose(107.341, 67.829),
                                    new Pose(82.317, 15.366)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(82.317, 15.366),
                                    new Pose(64.317, 92.195),
                                    new Pose(129.512, 83.415)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(129.512, 83.415), new Pose(97.024, 88.024))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50))
                    .build();
        }
    }





    /**
     * This is the state machine that controls the auto.
     */
    public int autonomousPathUpdate() {
        // 5. This is the state machine logic
        switch (pathState) {
            case 0: // Start Path 1 and rev shooter
                follower.followPath(paths.Path1); // Tell the follower to start
                setShooterRPM(3400.0, true); // Start revving shooter
                pathState++; // Move to the next state
                break;

            case 1: // Wait for Path 1 to finish
                if (!follower.isBusy()) {
                    pathState++; // Path is done, move on
                    // Set up for the shooting sequence
                    timer.reset(); // Reset timer for shooting window
                }
                break;

            case 2: // Align and Shoot for 8 seconds

                // Check if 8 seconds have passed
                if (timer.seconds() > 5.0) {
                    pathState++; // Time's up, move to the next state
                    break; // Exit the switch
                }

                // Run the intelligent, distance-based shooting sequence
                autonomousShootSequence();

                break; // Stay in this state until timer expires

            case 3: // Stop shooting, Start Path 2 and Intake
                stopShooter();
                runFeeder(0.0);
                runIntake(1.0); // Turn on intake
                follower.followPath(paths.Path2); // Start Path 2
                pathState++;
                break;

            case 4: // Wait for Path 2 to finish (while intaking)
                runIntake(1.0); // Keep intake on
                if (!follower.isBusy()) {
                    runIntake(0.0); // Stop intake
                    pathState++; // Path 2 is done
                }
                break;

            case 5: // Start Path 1 and rev shooter
                follower.followPath(paths.Path3); // Tell the follower to start
                setShooterRPM(3400.0, true); // Start revving shooter
                pathState++; // Move to the next state
                break;

            case 6: // Wait for Path 1 to finish
                if (!follower.isBusy()) {
                    pathState++; // Path is done, move on
                    // Set up for the shooting sequence
                    timer.reset(); // Reset timer for shooting window
                }
                break;

            case 7: // Align and Shoot for 8 seconds

                // Check if 8 seconds have passed
                if (timer.seconds() > 3.75) {
                    pathState++; // Time's up, move to the next state
                    break; // Exit the switch
                }

                // Run the intelligent, distance-based shooting sequence
                autonomousShootSequence();

                break; // Stay in this state until timer expires

            case 8: // Stop shooting, Start Path 2 and Intake
                stopShooter();
                runFeeder(0.0);
                runIntake(1.0); // Turn on intake
                follower.followPath(paths.Path4); // Start Path 2
                pathState++;
                break;

            case 9: // Wait for Path 2 to finish (while intaking)
                runIntake(1.0); // Keep intake on
                if (!follower.isBusy()) {
                    runIntake(0.0); // Stop intake
                    pathState++; // Path 2 is done
                }
                break;

            case 10: // Start Path 1 and rev shooter
                follower.followPath(paths.Path5); // Tell the follower to start
                setShooterRPM(3400.0, true);
                pathState++;
                break;

            case 11:
                if (!follower.isBusy()) {
                    pathState++; // Move to the next state
                    timer.reset();
                }
                break;
            case 12: // Align and Shoot for 8 seconds

                // Check if 8 seconds have passed
                if (timer.seconds() > 3.75) {
                    pathState++; // Time's up, move to the next state
                    break; // Exit the switch
                }

                // Run the intelligent, distance-based shooting sequence
                autonomousShootSequence();

                break; // Stay in this state until timer expires
            case 13: // Start Path 1 and rev shooter
                follower.followPath(paths.Path6); // Tell the follower to start
                setShooterRPM(0.0, true);
                runIntake(1.0);
                runFeeder(0.0);
                setShooterRPM(2800.0, true);
                pathState++;
                break;

            case 14:
                if (!follower.isBusy()) {
                    pathState++; // Move to the next state
                    timer.reset();
                }
                break;
            case 15: // Start Path 1 and rev shooter
                follower.followPath(paths.Path7); // Tell the follower to start
                runIntake(0.0);
                runFeeder(0.0);
                setShooterRPM(2800.0, true);
                pathState++;
                break;

            case 16:
                if (!follower.isBusy()) {
                    pathState++; // Move to the next state
                    timer.reset();
                }
                break;
            case 17:

                // Check if 8 seconds have passed
                if (timer.seconds() > 3.75) {
                    pathState++; // Time's up, move to the next state
                    break; // Exit the switch
                }

                // Run the intelligent, distance-based shooting sequence
                autonomousShootSequence();

                break; // Stay in this state until timer expires
            default:
                // Clean up all motors
                stopShooter();
                runFeeder(0.0);
                runIntake(0.0);
                follower.setTeleOpDrive(0, 0, 0, true); // Stop any rotation
                // Stop the OpMode
                requestOpModeStop();
                break;


        }
        return pathState;
    }

    /**
     * This private helper method contains all the logic for an intelligent
     * auto-shoot sequence. It is called repeatedly during the "shoot" state.
     */
    private void autonomousShootSequence() {
        // 1. Get current distance (using the inherited method from ExampleTeleOp2)
        double currentDistance = getDistance();

        // 2. Use your dynamic logic to determine shot parameters
        double targetRPM = 0.0;
        boolean useLongShotPID = false;
        double alignSetpoint = 0.0;

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
                targetRPM = 3400.0; // Fallback if no tag
            }
        }

        // --- 3. Alignment Logic ---
        autoAlignPID.setSetPoint(alignSetpoint); // Update PID setpoint
        OptionalDouble visionYaw = vision.getYaw(24);
        double rotation = 0.0;
        boolean atAlign = false;

        if (visionYaw.isPresent()) {
            rotation = -autoAlignPID.calculate(visionYaw.getAsDouble());
            atAlign = true; // Fire as long as tag is visible (per your change)
        }
        // (No scanning logic for auto, robot will just wait for tag)

        // Set drive power (0 for X/Y, calculated rotation for turn)
        follower.setTeleOpDrive(0, 0, rotation, true);

        // --- 4. Shooter Logic ---
        // Use the inherited public methods from ExampleTeleOp2
        setShooterRPM(targetRPM, useLongShotPID); // Rev up shooter to dynamic RPM
        boolean atRPM = isShooterAtRPMAuto(targetRPM); // Check against dynamic RPM

        // --- 5. Feeder Logic ---
        // Only feed if AT alignment AND AT RPM
        if (atRPM && atAlign) {
            runFeeder(-1.0);
            runIntake(1.0);
        } else {
            runFeeder(0.0);
            runIntake(0.0); // Stop intake if not shooting
        }
    }



    @Override
    public void stop() {
        // Ensure everything is off
        stopShooter();
        runFeeder(0.0);
        runIntake(0.0);
        follower.setTeleOpDrive(0, 0, 0, true);
        TeleopRED.startingPose = follower.getPose();
        super.stop(); // Call the parent stop method
    }
}

