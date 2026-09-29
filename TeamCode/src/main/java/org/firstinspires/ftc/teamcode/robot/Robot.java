package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.DriveConfig;
import org.firstinspires.ftc.teamcode.drivetrain.DifferentialDrive;
import org.firstinspires.ftc.teamcode.drivetrain.Drive;
import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.drivetrain.EncoderOdometry;
import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.drivetrain.PinpointOdometry;
import org.firstinspires.ftc.teamcode.drivetrain.Pose;

/**
 * The robot, assembled on demand.
 *
 * <p>Everything here is created the first time an OpMode asks for it, and never
 * before. That is the whole point: an OpMode that only wants to do a thing with the
 * drivetrain never touches the intake, and an OpMode that never asks for odometry
 * never spins up a Pinpoint. Nothing here can fail to initialize because of a module
 * another OpMode happened to reference.
 *
 * <pre>
 *   Drive drive = robot.drive();
 *   drive.goTo(72, 72, 90);
 * </pre>
 *
 * <h2>Why there is no dependency injection</h2>
 *
 * <p>Every FTC team shares one APK, so a module graph has to be built at runtime
 * anyway. A constructor taking a {@code Robot} is enough, and it keeps the call sites
 * looking like ordinary code instead of a framework. When a second mechanism shows up
 * and they need to talk to each other, this is the class that grows.
 */
public class Robot {

    /** Which mixing to build. Change this one line when the drivetrain changes. */
    public enum Chassis {
        MECANUM,
        DIFFERENTIAL
    }

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final DriveConfig config;
    private final Chassis chassis;

    private Drivetrain drivetrain;
    private Odometry odometry;
    private Drive drive;
    private Pose startPose;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, DriveConfig config, Chassis chassis) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.config = config;
        this.chassis = chassis;
    }

    // ------------------------------------------------------------------
    // Lazy modules
    // ------------------------------------------------------------------

    /** The drivetrain's motor mixing. Built on first use. */
    public Drivetrain drivetrain() {
        if (drivetrain == null) {
            drivetrain = chassis == Chassis.DIFFERENTIAL
                    ? new DifferentialDrive(hardwareMap, config)
                    : new MecanumDrive(hardwareMap, config);
            if (!drivetrain.isAvailable()) {
                telemetry.addData("Robot",
                        "WARNING: drivetrain is missing motors. Check the names in DriveConfig "
                                + "against hardware.xml.");
            }
        }
        return drivetrain;
    }

    /**
     * Where the robot thinks it is. Built on first use.
     *
     * <p>Prefers a Pinpoint and silently falls back to encoders if there is not one.
     * The fallback is a real fallback, not a stub: a robot with no Pinpoint wired up
     * still gets usable, if less accurate, odometry from its wheel encoders and the
     * hub IMU. That is deliberate, because the drivetrain is still being decided and
     * autonomous should be testable before the odometry computer is soldered in.
     */
    public Odometry odometry() {
        if (odometry == null) {
            if (config.usePinpoint) {
                PinpointOdometry pinpoint = new PinpointOdometry(hardwareMap, config);
                if (pinpoint.isAvailable()) {
                    odometry = pinpoint;
                }
            }
            if (odometry == null) {
                odometry = new EncoderOdometry(hardwareMap, drivetrain(), config);
            }
        }
        return odometry;
    }

    /** The high-level drivetrain. Built on first use, pulls in both of the above. */
    public Drive drive() {
        if (drive == null) {
            drive = new Drive(drivetrain(), odometry(), config);
        }
        return drive;
    }

    // ------------------------------------------------------------------
    // Lifecycle
    // ------------------------------------------------------------------

    /**
     * Declares where the robot is starting, in field coordinates.
     *
     * <p>Call this from {@code init()}. It is applied automatically, and only at the
     * right moment, by {@link #startUp()}. Do not apply it yourself: a Pinpoint that
     * is still calibrating will discard any position set before it finishes, and the
     * symptom is an autonomous that starts from (0, 0) no matter what you told it.
     */
    public void setStartPose(double xInches, double yInches, double headingDegrees) {
        this.startPose = new Pose(xInches, yInches, headingDegrees);
    }

    /**
     * Called once per loop from {@code init_loop()} while the robot is still held.
     *
     * <p>This is the only safe place to wait on a Pinpoint's calibration. An OpMode
     * needs a corrected starting position calls {@link #setStartPose} in {@code init()},
     * and this takes care of the rest.
     */
    public void startUp() {
        Odometry current = odometry();
        if (current instanceof PinpointOdometry) {
            PinpointOdometry pinpoint = (PinpointOdometry) current;
            if (pinpoint.isReady()) {
                // Recalibrate first, then apply the start pose. The other order
                // throws the pose away.
                pinpoint.recalibrate();
                if (startPose != null) {
                    pinpoint.setPose(startPose);
                }
            }
            return;
        }
        // The encoder fallback has nothing to calibrate, so apply it once and be done.
        if (startPose != null) {
            current.setPose(startPose);
            startPose = null;
        }
    }

    /** Called at the end of every OpMode. Stops the motors unconditionally. */
    public void shutDown() {
        if (drive != null) {
            drive.stop();
        } else if (drivetrain != null) {
            drivetrain.stop();
        }
    }

    public DriveConfig getConfig() {
        return config;
    }

    public Chassis getChassis() {
        return chassis;
    }

    public void addTelemetry(Telemetry telemetryTo) {
        telemetryTo.addData("Chassis", chassis);
        if (drive != null) {
            drive.addTelemetry(telemetryTo);
        } else {
            drivetrain().addTelemetry(telemetryTo);
            odometry().addTelemetry(telemetryTo);
        }
    }
}
