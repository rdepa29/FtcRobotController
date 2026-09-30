package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

// one drive motor, a missing motor is a no-op instead of a crash
//
// a DcMotorEx has no setPosition, STOP_AND_RESET_ENCODER is the only way to zero
//
// https://javadoc.io/doc/org.firstinspires.ftc/RobotCore/12.0.0/index.html
public class DriveMotor {

    private final String name;
    private final DcMotorEx motor;

    private DriveMotor(String name, DcMotorEx motor) {
        this.name = name;
        this.motor = motor;
    }

    // looks up a motor and applies its direction and zero power, absent instead
    // of throwing if the name is not configured
    public static DriveMotor create(
            HardwareMap hardwareMap, String name, DcMotor.Direction direction, ZeroPowerBehavior zero) {
        DcMotorEx motor = null;
        try {
            motor = hardwareMap.get(DcMotorEx.class, name);
        } catch (IllegalArgumentException notConfigured) {
            return absent(name);
        }
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(zero);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return new DriveMotor(name, motor);
    }

    // stands in for missing hardware, every command is a no-op
    public static DriveMotor absent(String name) {
        return new DriveMotor(name, null);
    }

    public String getName() {
        return name;
    }

    public boolean isPresent() {
        return motor != null;
    }

    // raw power, -1 to 1
    public void setPower(double power) {
        if (motor != null) {
            motor.setPower(power);
        }
    }

    // wheel velocity in ticks per second, needs an encoder run mode
    public void setVelocity(double ticksPerSecond) {
        if (motor != null) {
            motor.setVelocity(ticksPerSecond);
        }
    }

    public double getVelocity() {
        return motor == null ? 0.0 : motor.getVelocity();
    }

    public int getCurrentPosition() {
        return motor == null ? 0 : motor.getCurrentPosition();
    }

    public void setMode(DcMotor.RunMode mode) {
        if (motor != null) {
            motor.setMode(mode);
        }
    }

    // zeroes the encoder and restores the run mode
            return;
        }
        DcMotor.RunMode previous = motor.getMode();
        motor.setMotorDisable();
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(previous == DcMotor.RunMode.STOP_AND_RESET_ENCODER
                ? DcMotor.RunMode.RUN_WITHOUT_ENCODER
                : previous);
    }

    // call from OpMode.stop(), never from the loop
    public void stop() {
        if (motor != null) {
            motor.setPower(0.0);
        }
    }
}
