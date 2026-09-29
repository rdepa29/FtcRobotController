package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.config.DriveConfig;

/**
 * Odometry backed by a goBILDA Pinpoint computer.
 *
 * <p>The only class in the codebase that talks to the Pinpoint driver directly.
 * Every other file goes through {@link Odometry}, so if the driver's API ever moves,
 * this is the single file that needs editing.
 *
 * <h2>Set up order matters</h2>
 *
 * <p>Offsets, encoder resolution, and encoder directions all have to be written before
 * {@link GoBildaPinpointDriver#resetPosAndIMU()}, and the robot has to be stationary
 * while that reset happens. That is why the base OpMode exposes an init loop: it is
 * the only place where you can wait for the driver to finish calibrating without
 * eating into the 30 second autonomous period.
 */
public class PinpointOdometry implements Odometry {

    private final GoBildaPinpointDriver pinpoint;
    private final DriveConfig config;
    private final String deviceName;
    private Pose lastPose = new Pose(0, 0, 0);

    public PinpointOdometry(HardwareMap hardwareMap, DriveConfig config) {
        this.config = config;
        this.deviceName = config.pinpoint;
        this.pinpoint = find(hardwareMap);
        if (pinpoint != null) {
            configure();
        }
    }

    private GoBildaPinpointDriver find(HardwareMap hardwareMap) {
        try {
            return hardwareMap.get(GoBildaPinpointDriver.class, config.pinpoint);
        } catch (IllegalArgumentException notConfigured) {
            return null;
        }
    }

    /** Writes every setting the driver needs, then zeroes it. Robot must be stationary. */
    private void configure() {
        pinpoint.setOffsets(
                config.podXOffsetInches,
                config.podYOffsetInches,
                DistanceUnit.INCH);
        pinpoint.setEncoderResolution(config.odometryPods);
        pinpoint.setEncoderDirections(config.xEncoderDirection, config.yEncoderDirection);
        pinpoint.resetPosAndIMU();
    }

    /**
     * Re-zeroes the position and recalibrates the internal gyro.
     *
     * <p>Call from {@code init_loop()} with the robot still. Call it again any time
     * you know exactly where the robot is, to clear accumulated drift.
     */
    public void recalibrate() {
        if (pinpoint == null) {
            return;
        }
        pinpoint.recalibrateIMU();
        pinpoint.resetPosAndIMU();
    }

    /**
     * True once the driver has finished powering up and calibrating. An OpMode should
     * not call {@link #resetPose} before this, or the correction gets thrown away by
     * the calibration that follows.
     */
    public boolean isReady() {
        if (pinpoint == null) {
            return false;
        }
        GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
        return status == GoBildaPinpointDriver.DeviceStatus.READY;
    }

    @Override
    public void update() {
        if (pinpoint == null) {
            return;
        }
        pinpoint.update();
        lastPose = new Pose(
                pinpoint.getPosX(DistanceUnit.INCH),
                pinpoint.getPosY(DistanceUnit.INCH),
                pinpoint.getHeading(AngleUnit.DEGREES));
    }

    @Override
    public Pose getPose() {
        return lastPose;
    }

    @Override
    public double getXVelocity() {
        return pinpoint == null ? 0.0 : pinpoint.getVelX(DistanceUnit.INCH);
    }

    @Override
    public double getYVelocity() {
        return pinpoint == null ? 0.0 : pinpoint.getVelY(DistanceUnit.INCH);
    }

    @Override
    public double getYawVelocity() {
        if (pinpoint == null) {
            return 0.0;
        }
        return pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
    }

    @Override
    public void setPose(Pose pose) {
        lastPose = pose;
        if (pinpoint != null) {
            pinpoint.setPosition(pose.toPose2D());
        }
    }

    @Override
    public String getName() {
        return "pinpoint";
    }

    @Override
    public boolean isAvailable() {
        return pinpoint != null;
    }

    /** The raw driver, for anything this wrapper does not expose yet. */
    public GoBildaPinpointDriver getDriver() {
        return pinpoint;
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        if (pinpoint == null) {
            telemetry.addData("Odometry", "pinpoint (NOT CONFIGURED - '" + deviceName + "' missing)");
            return;
        }
        telemetry.addData("Odometry", "pinpoint");
        telemetry.addData("  Pose", lastPose.toString());
        GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
        telemetry.addData("  Status", status + (isReady() ? "" : "  (not ready)"));
        telemetry.addData("  Velocity", "%.1f, %.1f in/s", getXVelocity(), getYVelocity());
    }
}
