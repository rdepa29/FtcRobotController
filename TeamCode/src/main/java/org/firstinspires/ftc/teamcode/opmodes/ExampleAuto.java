package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;

/**
 * An example autonomous. The whole routine is four lines.
 *
 * <p>Nothing here is a real scoring path, because the robot is not designed yet. It
 * exists to prove the loop is closed: if these targets are hit repeatably and the pose
 * on telemetry is sane, the drivetrain, the odometry, and the control loop are working,
 * and every routine after this one is just more lines of the same shape.
 *
 * <h2>References</h2>
 *
 *  Autonomous structure:
 *    https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
 *
 *  Thirty seconds is the AUTO period, so device calibration cannot happen inside it.
 */
@Autonomous(name = "Drive: Example Auto", group = "01 Drive")
public class ExampleAuto extends RobotOpMode {

    /** Where the robot starts. Change this to match your actual starting tile. */
    private static final double START_X = 24.0;
    private static final double START_Y = 24.0;
    private static final double START_HEADING = 0.0;

    @Override
    protected void onInit() {
        telemetry.setMsTransmissionInterval(50);

        // Declared here, applied by the base class once the odometry is ready.
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
