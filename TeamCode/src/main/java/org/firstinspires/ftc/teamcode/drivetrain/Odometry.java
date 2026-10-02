package org.firstinspires.ftc.teamcode.drivetrain;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// where the robot thinks it is, separate from Drivetrain so it swaps cleanly
// first shipped the biobuzz tags with every field position at {0,0,0}
public interface Odometry {

    // reads new sensor data, once per loop before getPose
    void update();

    // current pose in inches and degrees
    Pose getPose();

    // inches per second along field +X
    // encoders report robot frame, so treat this as feedforward, never loop on it
    double getXVelocity();

    // same along field +Y
    double getYVelocity();

    // degrees per second, positive is CCW
    double getYawVelocity();

    // overrides the computed pose, apply the known start at the top of auto
    void setPose(Pose pose);

    // short name for telemetry
    String getName();

    // false if the hardware is missing or not answering
    boolean isAvailable();

    void addTelemetry(Telemetry telemetry);
}
