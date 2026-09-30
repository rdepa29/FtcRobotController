package org.firstinspires.ftc.teamcode.drivetrain;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// where the robot thinks it is, separate from Drivetrain so it can be swapped
// without touching the mixing
//
// first shipped the biobuzz apriltag clusters with every field position at
// {0,0,0}, so robotPose will not hand you an absolute position
//
// https://ftc-resources.firstinspires.org/ftc/game/manual
public interface Odometry {

    // reads new sensor data, once per loop before anything reads getPose
    void update();

    // current pose in inches and degrees
    Pose getPose();

    // inches per second along field +X
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
