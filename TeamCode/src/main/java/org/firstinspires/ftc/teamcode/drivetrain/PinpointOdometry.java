package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.config.DriveConfig;

// the only file that talks to the pinpoint driver directly, offsets and
// directions all have to be set before resetPosAndIMU and the robot has to be
// still while it happens, which is why it happens in init_loop
//
// podX is positive LEFT of center and podY positive FORWARD, rotating in
// place should hold within about 4in and if it swings a sign is wrong
//
// https://www.gobilda.com/content/user_manuals/3110-0002-0001%20User%20Guide.pdf
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

    // writes every setting then zeroes, robot has to be still
    private void configure() {
        pinpoint.setOffsets(
                config.podXOffsetInches,
                config.podYOffsetInches,
                DistanceUnit.INCH);
        pinpoint.setEncoderResolution(config.odometryPods);
        pinpoint.setEncoderDirections(config.xEncoderDirection, config.yEncoderDirection);
        pinpoint.resetPosAndIMU();
    }

    // re-zeroes and recalibrates the gyro
    public void recalibrate() {
        if (pinpoint == null) {
            return;
        }
        pinpoint.recalibrateIMU();
        pinpoint.resetPosAndIMU();
    }

    // true once the driver has powered up and calibrated, do not setPose before
    // this or the calibration that follows throws the correction away
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

    // raw driver, for anything this wrapper does not expose yet
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
