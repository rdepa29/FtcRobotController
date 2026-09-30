package org.firstinspires.ftc.teamcode.drivetrain;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.DriveConfig;
import org.firstinspires.ftc.teamcode.util.MathUtils;

// setPower and driveField drive by hand, goTo and turnTo send the robot
// somewhere and return immediately, setPower throws the target away and hands
// the wheels back
//
// field is 144x144in, heading 0 faces +Y, positive is CCW, full api in readme.md
//
// https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
public class Drive {

    private final Drivetrain drivetrain;
    private final Odometry odometry;
    private final DriveConfig config;

    // non-null while steering at a target
    private Pose target;
    private boolean requireHeading;
    private double xError;
    private double yError;
    private double headingError;

    public Drive(Drivetrain drivetrain, Odometry odometry, DriveConfig config) {
        this.drivetrain = drivetrain;
        this.odometry = odometry;
        this.config = config;
    }

    // direct control
    public void driveField(double forward, double strafe, double turn) {
        double heading = Math.toRadians(odometry.getPose().getHeading());
        double sin = Math.sin(heading);
        double cos = Math.cos(heading);
        // field frame to robot frame, this is the inverse of the rotation in update()
        // and the two are not the same expression, which is the easy thing to get wrong
        //
        // check it at heading 0, a field +Y command has to come out pure forward and
        // field +X pure strafe right
        setPower(
                forward * sin + strafe * cos,
                -forward * cos + strafe * sin,
                turn);
    }

    // stops the motors and drops the target, also runs at opmode exit
    public void stop() {
        cancel();
        drivetrain.stop();
    }

    // go there
    }

    // drives to a point and ends up facing a heading
        double forward = (xError * sin + yError * cos) * config.translationalGain;
        double strafe = (-xError * cos + yError * sin) * config.translationalGain;

        // heading gets a d term because a spinning robot overshoots badly and the
        // imu measures that well, position stays p only because wheel velocity is a
        // poor frame-ambiguous signal
        double turn = requireHeading
                ? headingError * config.headingGain
                        - odometry.getYawVelocity() * config.headingGainDamp
                : 0.0;

        turn = MathUtils.clamp(turn, -config.maxPower, config.maxPower);

        // normalize after the gains so a diagonal never exceeds maxPower
        double[] powers = {forward * config.autoPowerScale,
                strafe * config.autoPowerScale,
                turn * config.autoPowerScale};
        MathUtils.normalize(powers, config.maxPower);

        if (isAtTarget()) {
            stop();
            return;
        }
        drivetrain.drive(powers[0], powers[1], powers[2]);
    }

    // true while a goTo is still working
    public boolean isBusy() {
        return target != null;
    }

    private boolean isAtTarget() {
        double positionError = Math.sqrt(xError * xError + yError * yError);
        if (positionError > config.positionToleranceInches) {
            return false;
        }
        return !requireHeading || Math.abs(headingError) <= config.headingToleranceDegrees;
    }

    // forgets the target and hands the wheels back
    public void cancel() {
        target = null;
        xError = 0.0;
        yError = 0.0;
        headingError = 0.0;
    }

    // reading the robot
    public Pose getPose() {
        return odometry.getPose();
    }

    // overrides the computed pose, use at auto start
    public void setPose(double xInches, double yInches, double headingDegrees) {
        odometry.setPose(new Pose(xInches, yInches, headingDegrees));
    }

    // overrides the computed pose
    public void setPose(Pose pose) {
        odometry.setPose(pose);
    }

    // zeroes the encoders and puts the pose back at (0,0)
        drivetrain.resetEncoders();
        odometry.setPose(new Pose(0, 0, 0));
    }

    // inches to the target, 0 if there is none
    public double getPositionError() {
        return Math.sqrt(xError * xError + yError * yError);
    }

    // degrees to the target heading, 0 if there is none
    public double getHeadingError() {
        return headingError;
    }

    public Drivetrain getDrivetrain() {
        return drivetrain;
    }

    public Odometry getOdometry() {
        return odometry;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Pose", "x %.1f  y %.1f  h %.1f",
                getPose().getX(), getPose().getY(), getPose().getHeading());
        if (isBusy()) {
            telemetry.addData("Target", "%.1f in / %.1f deg away",
                    getPositionError(), getHeadingError());
        }
        drivetrain.addTelemetry(telemetry);
        odometry.addTelemetry(telemetry);
    }
}
