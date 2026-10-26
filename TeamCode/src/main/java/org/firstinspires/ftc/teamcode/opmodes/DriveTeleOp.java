package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.config.DriverInput;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;

@TeleOp(name = "Drive: TeleOp", group = "01 Drive")
public class DriveTeleOp extends RobotOpMode {

    private final DriverInput input = new DriverInput(RobotConfig.ROBOT);
    // init -> robot relative
    private boolean fieldRelative;
    private boolean toggled;

    @Override
    protected void onInit() {
        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Sticks: " + input.mapping());
        telemetry.addLine("LB toggles robot-relative / field-relative");
        telemetry.addLine("LS click zeroes the pose");
    }

    @Override
    protected void onLoop() {
        input.update(gamepad1);

        //
        if (gamepad1.left_bumper && !toggled) {
            fieldRelative = !fieldRelative;
            toggled = true;
        } else if (!gamepad1.left_bumper) {
            toggled = false;
        }

        if (fieldRelative) {
            drive().driveField(input.forward(), input.strafe(), input.turn());
        } else {
            drive().setPower(input.forward(), input.strafe(), input.turn());
        }

        if (gamepad1.left_stick_button) {
            drive().resetEncodersAndPose();
        }

        telemetry.addData("Mode", fieldRelative ? "field-relative" : "robot-relative");
        telemetry.addData("Drive", "%+.2f %+.2f %+.2f",
                input.forward(), input.strafe(), input.turn());
        publishTelemetry();
    }
}
