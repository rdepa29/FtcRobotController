package org.firstinspires.ftc.teamcode.drivetrain;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.DriveConfig;
import org.firstinspires.ftc.teamcode.util.MathUtils;

// fallback odometry from the drive encoders plus the hub imu, which is in the
// hardware map as "imu" on either hub, worse than a pinpoint in every way so
// keep it a backup
//
// reads positions and never touches run mode, commanding power to a motor in
// RUN_USING_ENCODER is an error
//
// https://javadoc.io/doc/org.firstinspires.ftc/RobotCore/12.0.0/index.html
public class EncoderOdometry implements Odometry {

    private final DriveMotor[] motors;
    private final IMU imu;
    private final double ticksPerInch;
    private final ElapsedTime timer = new ElapsedTime();

    private int[] lastTicks = new int[4];
    private double lastHeading = 0.0;
    private Pose pose = new Pose(0, 0, 0);
    private double xVelocity;
    private double yVelocity;
    private double yawVelocity;

    public EncoderOdometry(HardwareMap hardwareMap, Drivetrain drivetrain, DriveConfig config) {
        this.motors = drivetrain.getMotors();
        this.imu = findImu(hardwareMap);

        double ticksPerRevolution = config.motorTicksPerRevolution * config.gearRatio;
        this.ticksPerInch = ticksPerRevolution / (2.0 * Math.PI * config.wheelRadiusInches);

        resetTicks();
    }

    private IMU findImu(HardwareMap hardwareMap) {
        try {
            return hardwareMap.get(IMU.class, "imu");
        } catch (IllegalArgumentException notConfigured) {
            return null;
        }
    }

    private void resetTicks() {
        lastTicks = new int[motors.length];
        for (int i = 0; i < motors.length; i++) {
            lastTicks[i] = motors[i].getCurrentPosition();
        }
    }

    @Override
    public void update() {
        double dt = timer.seconds();
        if (dt <= 0.0) {
            return;
        }
        timer.reset();

        int[] ticks = new int[motors.length];
        for (int i = 0; i < motors.length; i++) {
            ticks[i] = motors[i].getCurrentPosition();
        }

        double frontLeft = inchesSinceLast(ticks[0], 0);
        double frontRight = inchesSinceLast(ticks[1], 1);
        double backLeft = inchesSinceLast(ticks[2], 2);
        double backRight = inchesSinceLast(ticks[3], 3);

        // standard mecanum inverse mixing, derive it from the mecanum formula rather
        // than guessing, forward is all four averaged and the lateral term
        // alternates sign across the diagonals
        //
        // check it by making a pure left strafe come out zero forward and negative
        // rightward, and make sure all four wheels report positive going forward, a
        // wrong direction belongs in DriveConfig and not here
        double forward = (frontLeft + frontRight + backLeft + backRight) / 4.0;
        double right = (-frontLeft + frontRight + backLeft - backRight) / 4.0;

        double heading = readHeading();
        double yawDelta = MathUtils.headingError(lastHeading, heading);
        double sin = Math.sin(Math.toRadians(heading));
        double cos = Math.cos(Math.toRadians(heading));

        // robot frame into field frame, at heading 0 this gives dx = -right and
        // dy = forward
        double dx = forward * sin - right * cos;
        double dy = forward * cos - right * sin;

        pose = new Pose(pose.getX() + dx, pose.getY() + dy, heading);

        xVelocity = dx / dt;
        yVelocity = dy / dt;
        yawVelocity = yawDelta / dt;

        lastTicks = ticks;
        lastHeading = heading;
    }

    private double inchesSinceLast(int currentTicks, int motorIndex) {
        return (currentTicks - lastTicks[motorIndex]) / ticksPerInch;
    }

    // hub imu yaw, ZYX puts yaw first and it is the only angle needed
    private double readHeading() {
        if (imu == null) {
            return 0.0;
        }
        return imu.getRobotOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    @Override
    public Pose getPose() {
        return pose;
    }

    @Override
    public double getXVelocity() {
        return xVelocity;
    }

    @Override
    public double getYVelocity() {
        return yVelocity;
    }

    @Override
    public double getYawVelocity() {
        return yawVelocity;
    }

    @Override
    public void setPose(Pose newPose) {
        pose = newPose;
        lastHeading = newPose.getHeading();
        if (imu != null) {
            imu.resetYaw();
        }
        resetTicks();
    }

    @Override
    public String getName() {
        return "encoders";
    }

    @Override
    public boolean isAvailable() {
        return imu != null;
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Odometry",
                "encoders" + (isAvailable() ? "" : " (NO IMU - heading unusable)"));
        telemetry.addData("  Pose", pose.toString());
        telemetry.addData("  Velocity", "%.1f, %.1f in/s", xVelocity, yVelocity);
    }
}
