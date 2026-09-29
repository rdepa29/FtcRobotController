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

/**
 * Odometry computed from the drive motor encoders plus the onboard IMU.
 *
 * <p>This is the fallback when no Pinpoint is wired up, and it is good enough to tune
 * autonomous on. It is worse than dead reckoning in every respect, so prefer a
 * Pinpoint when you have one: wheel slip under load is invisible to this, and the
 * position error is a random walk that grows all match long.
 *
 * <p>Both the Control Hub and the Expansion Hub expose an IMU in the hardware map
 * under the name {@code imu}, so this costs no extra hardware. Without an IMU there is
 * no way to know the robot's heading from wheel encoders alone, so this class reports
 * itself unavailable rather than quietly returning a heading that never changes.
 *
 * <h2>Why the motors stay in RUN_WITHOUT_ENCODER</h2>
 *
 * <p>Encoder <em>positions</em> are readable in any run mode. Commanding power to a
 * motor that is in {@code RUN_USING_ENCODER} is an error, and mixing the two is the
 * easiest way to end up with drivetrain motors that refuse to move. This class
 * deliberately reads positions and never changes the mode the drivetrain set up.
 *
 * <h2>References</h2>
 *
 *  The fallback used when no Pinpoint is wired up. The hub IMU appears in the
 *  hardware map under the name imu on both the Control Hub and the Expansion Hub:
 *    https://javadoc.io/doc/org.firstinspires.ftc/RobotCore/12.0.0/index.html
 *
 *  Yaw is read as firstAngle with AxesOrder.ZYX. Orientation has no
 *  getYaw(AngleUnit) method; firstAngle is a public field.
 */
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

        // Standard mecanum inverse mixing, recovering the body-frame velocities the
        // four wheel positions imply. Derive these from MecanumDrive's formula rather
        // than guessing: forward = all four averaged, and the lateral term alternates
        // sign across the diagonals. Getting this wrong is silent, so the check is
        // that a pure left strafe comes out as zero forward and negative rightward.
        //
        // This assumes motor directions are configured so that all four wheels report
        // positive counts when the robot moves forward. If a move is tracked with the
        // wrong sign, fix the direction in DriveConfig rather than the sign here.
        double forward = (frontLeft + frontRight + backLeft + backRight) / 4.0;
        double right = (-frontLeft + frontRight + backLeft - backRight) / 4.0;

        double heading = readHeading();
        double yawDelta = MathUtils.headingError(lastHeading, heading);
        double sin = Math.sin(Math.toRadians(heading));
        double cos = Math.cos(Math.toRadians(heading));

        // Rotate the robot-frame motion into the field frame, using the 0 deg = +Y
        // heading convention on Pose. At heading 0 this gives dx = -right, dy = forward.
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

    /**
     * Reads the hub IMU. With {@link AxesOrder#ZYX} the first angle is the yaw, which
     * is the only one this class needs.
     */
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
