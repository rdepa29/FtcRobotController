package org.firstinspires.ftc.teamcode.drivetrain;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.DriveConfig;
import org.firstinspires.ftc.teamcode.util.MathUtils;

/**
 * Four-wheel differential (tank) mixing.
 *
 * <p>Included because the drivetrain type is exactly the thing that changes most often
 * in a season, and it should be a one-line change in the OpMode rather than a rewrite.
 * Everything above this class, including {@link Drive} and every motion, works
 * unchanged on a differential base.
 *
 * <p>Strafing is not physically available, so {@code roll} is accepted and discarded.
 * A {@link Drive} on a differential base will simply fail to strafe, which is the
 * correct behavior; it will not silently spin.
 */
public class DifferentialDrive implements Drivetrain {

    private final DriveMotor frontLeft;
    private final DriveMotor frontRight;
    private final DriveMotor backLeft;
    private final DriveMotor backRight;
    private final DriveConfig config;
    private final double[] powers = new double[4];

    public DifferentialDrive(HardwareMap hardwareMap, DriveConfig config) {
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
        // roll is intentionally unused; see the class comment.
        double left = pitch - yaw;
        double right = pitch + yaw;

        powers[0] = left;
        powers[1] = right;
        powers[2] = left;
        powers[3] = right;

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
        return "differential";
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
