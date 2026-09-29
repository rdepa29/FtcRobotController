package org.firstinspires.ftc.teamcode.drivetrain;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.DriveConfig;
import org.firstinspires.ftc.teamcode.util.MathUtils;

/**
 * Four-wheel mecanum mixing.
 *
 * <p>This is the one place the mixing formula lives. The three cases worth checking
 * against reality before you trust the robot:
 *
 * <pre>
 *   forward only   (0,0,0 -> 1,0,0)  all four motors equal
 *   turn left      (0,0,0 -> 0,0,1)  left motors back, right motors forward
 *   strafe right   (0,0,0 -> 0,1,0)  diagonal pairs opposed
 * </pre>
 *
 * <p>If the third one goes the wrong way, that is not a software bug. Mecanum rollers
 * are laid out in one of two mirror-image arrangements and yours may be the other one;
 * set {@link DriveConfig#mirrorStrafe} rather than editing the formula.
 */
public class MecanumDrive implements Drivetrain {

    private final DriveMotor frontLeft;
    private final DriveMotor frontRight;
    private final DriveMotor backLeft;
    private final DriveMotor backRight;
    private final DriveConfig config;
    private final double[] powers = new double[4];

    public MecanumDrive(HardwareMap hardwareMap, DriveConfig config) {
        this.config = config;
        this.frontLeft = DriveMotor.create(hardwareMap, config.frontLeft,
                config.frontLeftDirection, config.zeroPowerBehavior);
        this.frontRight = DriveMotor.create(hardwareMap, config.frontRight,
                config.frontRightDirection, config.zeroPowerBehavior);
        this.backLeft = DriveMotor.create(hardwareMap, config.backLeft,
                config.backLeftDirection, config.zeroPowerBehavior);
        this.backRight = DriveMotor.create(hardwareMap, config.backRight,
                config.backRightDirection, config.zeroPowerBehavior);
    }

    @Override
    public void drive(double pitch, double roll, double yaw) {
        double lateral = config.mirrorStrafe ? -roll : roll;

        powers[0] = pitch + lateral - yaw;
        powers[1] = pitch - lateral + yaw;
        powers[2] = pitch - lateral - yaw;
        powers[3] = pitch + lateral + yaw;

        MathUtils.normalize(powers, config.maxPower);

        frontLeft.setPower(powers[0]);
        frontRight.setPower(powers[1]);
        backLeft.setPower(powers[2]);
        backRight.setPower(powers[3]);
    }

    @Override
    public void stop() {
        powers[0] = powers[1] = powers[2] = powers[3] = 0.0;
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }

    @Override
    public void resetEncoders() {
        frontLeft.resetEncoder();
        frontRight.resetEncoder();
        backLeft.resetEncoder();
        backRight.resetEncoder();
    }

    @Override
    public DriveMotor[] getMotors() {
        return new DriveMotor[]{frontLeft, frontRight, backLeft, backRight};
    }

    @Override
    public String getName() {
        return "mecanum";
    }

    @Override
    public boolean isAvailable() {
        return frontLeft.isPresent() && frontRight.isPresent()
                && backLeft.isPresent() && backRight.isPresent();
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Drivetrain", getName() + (isAvailable() ? "" : " (MISSING MOTORS)"));
        telemetry.addData("  FL / FR", "%.2f / %.2f", powers[0], powers[1]);
        telemetry.addData("  BL / BR", "%.2f / %.2f", powers[2], powers[3]);
    }
}
