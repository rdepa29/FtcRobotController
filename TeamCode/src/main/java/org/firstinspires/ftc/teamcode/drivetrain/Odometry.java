package org.firstinspires.ftc.teamcode.drivetrain;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Where the robot thinks it is on the field.
 *
 * <p>Kept separate from {@link Drivetrain} on purpose. The drivetrain decides how the
 * wheels turn; odometry decides where that puts us. Swapping a Pinpoint for dead
 * reckoning from the wheel encoders, or later fusing in a camera correction, must not
 * require touching the mixing code or the motions.
 *
 * <p>This season it matters more than usual: the SDK ships all four BIOBUZZ AprilTag
 * clusters, but FIRST published their field positions as all zeros, so
 * {@code AprilTagDetection.robotPose} will not give you an absolute field position
 * out of the box. Odometry is the only source of absolute position, which is why
 * {@link PinpointOdometry} is the default and not an optional extra.
 *
 * <h2>References</h2>
 *
 *    https://www.gobilda.com/content/user_manuals/3110-0002-0001%20User%20Guide.pdf
 *    https://github.com/goBILDA-Official/FtcRobotController-Add-Pinpoint
 *
 *  Important for BIOBUZZ: the SDK ships all four field AprilTag clusters, but FIRST
 *  published every cluster's field position as {0,0,0} with identity orientation.
 *  AprilTagDetection.robotPose will therefore NOT return a usable absolute field
 *  position out of the box, which makes odometry the only reliable source. That is
 *  why PinpointOdometry is the default and not an optional extra. See manual 9.9.
 */
public interface Odometry {

    /**
     * Reads new sensor data. Call once per loop, before anything reads
     * {@link #getPose()}. The base OpMode does this for you.
     */
    void update();

    /** The robot's current pose, in inches and degrees. */
    Pose getPose();

    /**
     * Velocity along the field's +X axis, in inches per second.
     *
     * <p>Read with care. Wheel encoders and most IMU-based odometry devices report
     * velocity in the robot's own frame, not the field's, and this interface does not
     * hide that. Treat velocity as diagnostic telemetry and optional feedforward only;
     * do not build a control loop that depends on it being field-relative.
     */
    double getXVelocity();

    /** Velocity along the field's +Y axis, in inches per second. See {@link #getXVelocity()}. */
    double getYVelocity();

    /** Angular velocity, positive counter-clockwise, in degrees per second. */
    double getYawVelocity();

    /**
     * Tells the odometry where the robot is, overriding whatever it had computed.
     * Use at the start of autonomous to apply the known starting position.
     */
    void setPose(Pose pose);

    /** Short name for telemetry, e.g. "pinpoint". */
    String getName();

    /** False when the underlying hardware is missing or not answering. */
    boolean isAvailable();

    void addTelemetry(Telemetry telemetry);
}
