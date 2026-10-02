package org.firstinspires.ftc.teamcode.drivetrain;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.DriveConfig;
import org.firstinspires.ftc.teamcode.util.MathUtils;

// manual or non-blocking goTo, field is 144x144in
// heading 0 faces +Y, positive is CCW, api in readme.md
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

    /// direct control
    // -1 to 1, forward, strafe left, turn CCW, cancels any goTo
    public void setPower(double forward, double strafe, double turn) {
        cancel();
        drivetrain.drive(forward, strafe, turn);
    }

    // same as setPower but field-relative, stick forward always means field +Y
    public void driveField(double forward, double strafe, double turn) {
        double heading = Math.toRadians(odometry.getPose().getHeading());
        double sin = Math.sin(heading);
        double cos = Math.cos(heading);
        // field frame to robot frame, inverse of the rotation in update()
        // check at heading 0, field +Y pure forward, +X pure strafe right
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

    /// go there
    // drives to a point and keeps the heading it arrives with
    public void goTo(double xInches, double yInches) {
        this.target = new Pose(xInches, yInches, odometry.getPose().getHeading());
        this.requireHeading = false;
    }

    // drives to a point facing a heading, non-blocking
    // inside positionToleranceInches already counts as there
    public void goTo(double xInches, double yInches, double headingDegrees) {
        this.target = new Pose(xInches, yInches, MathUtils.wrapDegrees(headingDegrees));
        this.requireHeading = true;
    }

    // drives to a pose, using its heading
    public void goTo(Pose pose) {
        goTo(pose.getX(), pose.getY(), pose.getHeading());
    }

    // turns in place to a heading, holds with a goTo to the current spot
    // not for short moves
    public void turnTo(double headingDegrees) {
        Pose pose = odometry.getPose();
        goTo(pose.getX(), pose.getY(), headingDegrees);
    }

    // turns by a number of degrees, positive is left
    public void turnBy(double degrees) {
        turnTo(odometry.getPose().getHeading() + degrees);
    }

    /// state
    // refreshes odometry and steers at the target, once per loop
    public void update() {
        odometry.update();
        if (target == null) {
            return;
        }

        Pose pose = odometry.getPose();
        xError = target.getX() - pose.getX();
        yError = target.getY() - pose.getY();
        headingError = MathUtils.headingError(pose.getHeading(), target.getHeading());

        // rotate the field error into the robot frame so zero error is always
        // zero power
        double heading = Math.toRadians(pose.getHeading());
        double sin = Math.sin(heading);
        double cos = Math.cos(heading);
        double forward = (xError * sin + yError * cos) * config.translationalGain;
        double strafe = (-xError * cos + yError * sin) * config.translationalGain;

        // heading gets a d term, spinning overshoots and the imu sees it
        // position stays p, wheel velocity is a poor signal
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

    /// reading the robot
    // current pose in inches and degrees
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

    // zeroes the encoders and puts the pose back at (0,0), teleop only and
    // never during a match because it throws away the position odometry earned
    public void resetEncodersAndPose() {
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
