package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.config.DriveConfig;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;
import org.firstinspires.ftc.teamcode.util.MathUtils;

/**
 * The one TeleOp to start with. It exercises the whole drivetrain and nothing else.
 *
 * <p>Drive sticks: forward/back on the left stick, strafe on the right stick's X,
 * turn on the right stick's Y. Hold the left bumper to switch between robot-relative
 * and field-relative driving, which is the fastest way to convince yourself why
 * field-relative is nicer.
 *
 * <h2>References</h2>
 *
 *  Gamepad input fields and stick conventions:
 *    https://javadoc.io/doc/org.firstinspires.ftc/RobotCore/12.0.0/index.html
 *
 *  Note that Gamepad exposes right_stick_x and right_stick_y as flat float fields,
 *  not as a nested right_stick object.
 */
@TeleOp(name = "Drive: TeleOp", group = "01 Drive")
public class DriveTeleOp extends RobotOpMode {

    private double fieldRelative = 1.0;
    private boolean toggled;

    @Override
    protected void onInit() {
        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Left stick: forward/back    Right stick X: strafe    Y: turn");
        telemetry.addLine("LB toggles robot-relative / field-relative");
    }

    @Override
    protected void onLoop() {
        double forward = gamepad1.left_stick_y;
        double strafe = -gamepad1.right_stick_x;
        double turn = -gamepad1.right_stick_y;

        if (gamepad1.left_bumper && !toggled) {
            fieldRelative = -fieldRelative;
            toggled = true;
        } else if (!gamepad1.left_bumper) {
            toggled = false;
        }

        forward = MathUtils.applyDeadZone(forward, DriveConfig.ROBOT.deadZone);
        strafe = MathUtils.applyDeadZone(strafe, DriveConfig.ROBOT.deadZone);
        turn = MathUtils.applyDeadZone(turn, DriveConfig.ROBOT.deadZone);

        if (fieldRelative > 0) {
            drive().driveField(forward, strafe, turn);
        } else {
            drive().setPower(forward, strafe, turn);
        }

        if (gamepad1.left_stick_button) {
            drive().resetEncodersAndPose();
        }

        telemetry.addData("Mode", fieldRelative > 0 ? "field-relative" : "robot-relative");
        publishTelemetry();
    }
}
