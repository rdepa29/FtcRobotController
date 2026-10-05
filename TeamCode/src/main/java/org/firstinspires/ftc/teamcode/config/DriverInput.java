package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.util.MathUtils;

/// reads a gamepad into forward, strafe and turn
// the mapping lives in DriveConfig, this only follows it
public class DriverInput {

    private final DriveConfig config;
    private double forward;
    private double strafe;
    private double turn;

    public DriverInput(DriveConfig config) {
        this.config = config;
    }

    /// call once per loop before the values are read
    // y up is forward, x right is negative strafe and negative turn
    public void update(Gamepad pad) {
        forward = 0.0;
        strafe = 0.0;
        turn = 0.0;

        apply(pad.left_stick_x, config.leftX);
        apply(pad.left_stick_y, config.leftY);
        apply(pad.right_stick_x, config.rightX);
        apply(pad.right_stick_y, config.rightY);

        forward = MathUtils.applyDeadZone(forward, config.deadZone);
        strafe = MathUtils.applyDeadZone(strafe, config.deadZone);
        turn = MathUtils.applyDeadZone(turn, config.deadZone);
    }

    // two axes on the same motion add up, so one axis per motion is cleanest
    private void apply(double raw, StickMotion motion) {
        switch (motion) {
            case Move:
                forward += raw;
                break;
            case Strafe:
                strafe -= raw;
                break;
            case Turn:
                turn -= raw;
                break;
            case None:
            default:
                break;
        }
    }

    // forward positive, strafe positive is robot-left, turn positive is CCW
    public double forward() {
        return forward;
    }

    public double strafe() {
        return strafe;
    }

    public double turn() {
        return turn;
    }

    // "LX:Turn  LY:None  RX:Strafe  RY:Move", so a remap is visible on the DS
    public String mapping() {
        return "LX:" + config.leftX + "  LY:" + config.leftY
                + "  RX:" + config.rightX + "  RY:" + config.rightY;
    }
}