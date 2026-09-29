package org.firstinspires.ftc.teamcode.drivetrain;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Translates three driver-style axes into motor powers.
 *
 * <p>This is deliberately the narrowest interface in the codebase. A drivetrain's only
 * job is mixing; it has no opinion about the field, odometry, or where the robot is
 * trying to go. All of that lives in {@link Drive}, which is what lets you swap
 * mecanum for differential without touching a single OpMode.
 *
 * <h2>Axis conventions, used by every implementation</h2>
 * <ul>
 *   <li><b>pitch</b> positive moves the robot forward.</li>
 *   <li><b>roll</b> positive strafes the robot to its own right.</li>
 *   <li><b>yaw</b> positive turns the robot counter-clockwise.</li>
 * </ul>
 *
 * <p>Getting these three right is the whole job of this interface. If the robot turns
 * the wrong way, fix the sign here, not in the OpMode.
 *
 * <h2>References</h2>
 *
 *  The mixing interface, kept deliberately narrow so a change of drivetrain never has
 *  to reach the control loop above it:
 *    https://gm0.org/en/latest/docs/common-mechanisms/drivetrains/holonomic.html
 *    https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
 */
public interface Drivetrain {

    /**
     * Applies a robot-relative movement command. Values are typically -1 to 1 and are
     * normalized internally, so passing (1, 1, 1) is legal and simply drives diagonally
     * and turns at full speed.
     */
    void drive(double pitch, double roll, double yaw);

    /** Commands zero power. Does not reset encoders. */
    void stop();

    /** Zeroes the motor encoders. Only meaningful for encoder-based odometry. */
    void resetEncoders();

    /** The four motors, ordered front-left, front-right, back-left, back-right. */
    DriveMotor[] getMotors();

    /** Short name for telemetry, e.g. "mecanum". */
    String getName();

    /** False when any required motor is missing from the hardware map. */
    boolean isAvailable();

    void addTelemetry(Telemetry telemetry);
}
