package org.firstinspires.ftc.teamcode.Drivetrain;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Drivetrain {
    Odometry odometry;

    public Drivetrain(
        Odometry odometry
    ) {
        this.odometry = odometry;
    }

    public void GoTo(Pose2D position) {
        double x = odometry.getX();
        double y = odometry.getY();

        double dx = position.getX() - x;
        double dy = position.getY() - y;
    }
}
