package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

/**
 * One drive motor, wrapped so that missing hardware is survivable.
 *
 * <p>If the device named in the config is not in the hardware map, {@link #isPresent()}
 * returns false and every command on this object does nothing. A robot that is missing
 * a wheel then drives in a straight-ish line and still finishes the OpMode, instead of
 * throwing out of {@code hardwareMap.get} and taking the whole run with it.
 *
 * <p>No OpenCV, no OpMode state, no timing. Safe to construct in a unit test only if
 * you use the {@link #absent(String)} factory instead of the hardware map one.
 */
public class DriveMotor {

    private final String name;
    private final DcMotorEx motor;

    private DriveMotor(String name, DcMotorEx motor) {
        this.name = name;
        this.motor = motor;
    }

    /**
     * Looks up a motor and applies its direction and zero-power behavior.
     *
     * <p>Returns an absent motor rather than throwing if the name is not configured.
     */
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

    /** A stand-in for hardware that is not present. All commands become no-ops. */
    public static DriveMotor absent(String name) {
        return new DriveMotor(name, null);
    }

    public String getName() {
        return name;
    }

    public boolean isPresent() {
        return motor != null;
    }

    /** Raw motor power, -1 to 1. No-op if the motor is absent. */
    public void setPower(double power) {
        if (motor != null) {
            motor.setPower(power);
        }
    }

    /** Commanded wheel velocity in ticks per second. Requires an encoder run mode. */
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

    /**
     * Zeroes the encoder.
     *
     * <p>There is no {@code setPosition} on a {@link com.qualcomm.robotcore.hardware.DcMotorEx}.
     * {@link DcMotor.RunMode#STOP_AND_RESET_ENCODER} is the only supported way to do
     * this, and the motor ends up stopped as a side effect, so the mode is restored
     * to whatever the drivetrain runs in.
     */
    public void resetEncoder() {
        if (motor == null) {
            return;
        }
        DcMotor.RunMode previous = motor.getMode();
        motor.setMotorDisable();
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(previous == DcMotor.RunMode.STOP_AND_RESET_ENCODER
                ? DcMotor.RunMode.RUN_WITHOUT_ENCODER
                : previous);
    }

    /** Zeroes the motor. Call this from OpMode.stop(), never from the loop. */
    public void stop() {
        if (motor != null) {
            motor.setPower(0.0);
        }
    }
}
