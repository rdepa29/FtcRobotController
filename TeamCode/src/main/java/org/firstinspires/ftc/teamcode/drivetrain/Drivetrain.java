package org.firstinspires.ftc.teamcode.drivetrain;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// three driver axes in, motor powers out, nothing else
// pitch forward roll right yaw CCW, mecanum and differential both fit
public interface Drivetrain {

    // robot-relative, normalized internally so (1,1,1) is legal
    void drive(double pitch, double roll, double yaw);

    // zero power, encoders untouched
    void stop();

    // zeroes the encoders, only matters for encoder odometry
    void resetEncoders();

    // fl, fr, bl, br in that order
    DriveMotor[] getMotors();

    // short name for telemetry
    String getName();

    // false if any motor is missing
    boolean isAvailable();

    void addTelemetry(Telemetry telemetry);
}
