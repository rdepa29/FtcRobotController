package org.firstinspires.ftc.teamcode.drivetrain;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// three driver axes in, motor powers out, nothing else, no field, no
// odometry and no opinion where the robot is going, which is what lets
// mecanum swap for differential without touching an opmode
//
// pitch forward, roll right, yaw CCW
//
// https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
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
