package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;

// not a scoring path, it exists to prove the loop is closed
// if these four targets land repeatably and the pose looks sane
// the drivetrain and odometry work
@Autonomous(name = "Drive: Example Auto", group = "01 Drive")
public class ExampleAuto extends RobotOpMode {

    // where the robot starts, change to match your actual starting tile
    private static final double START_X = 24.0;
    private static final double START_Y = 24.0;
    private static final double START_HEADING = 0.0;

    @Override
    protected void onInit() {
        telemetry.setMsTransmissionInterval(50);

        // declared here, applied by the base class once the odometry is ready
        setStartPose(START_X, START_Y, START_HEADING);

        sequence()
                .add("to first point", START_X + 24.0, START_Y, 0.0)
                .add("strafe across", START_X + 24.0, START_Y + 36.0, 0.0)
                .add("turn around", START_X + 24.0, START_Y + 36.0, 180.0)
                .add("come home", START_X, START_Y, START_HEADING);
    }

    @Override
    protected void onLoop() {
        sequence().run();
        publishTelemetry();
    }
}
