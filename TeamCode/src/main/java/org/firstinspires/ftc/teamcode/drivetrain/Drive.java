package org.firstinspires.ftc.teamcode.drivetrain;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.DriveConfig;
import org.firstinspires.ftc.teamcode.util.MathUtils;

/**
 * The drivetrain you actually talk to. One object, two modes, nothing else.
 *
 * <h2>Mode 1: "move that way, this fast" (TeleOp)</h2>
 *
 * <pre>
 * drive.setPower(forward, strafe, turn);   // robot-relative, each -1..1
 * drive.driveField(forward, strafe, turn); // same, but push forward = field +Y
 * </pre>
 *
 * <h2>Mode 2: "go there" (Autonomous)</h2>
 *
 * <pre>
 *   drive.goTo(72, 72);                          // drive to a point on the field
 *   drive.goTo(72, 72, 0);                       // ...and end up facing 0 degrees
 *   drive.goTo(FieldConfig.cell(Tile.C3));       // ...or a named place
 *   drive.goTo(FieldConfig.cell(Tile.C3, 90));
 *
 *   drive.turnTo(90);                            // face 90 degrees
 *   drive.turnBy(-45);                           // turn 45 degrees right
 *
 *   if (!drive.isBusy()) { nextStep(); }
 * </pre>
 *
 * <h2>How the two modes coexist</h2>
 *
 * <p>There is exactly one set of wheels, so exactly one thing drives them at a time.
 * {@link #goTo} sets a target and {@link #update()} steers toward it. Calling
 * {@link #setPower} throws the target away and hands the wheels straight back to the
 * driver. There is no scheduler, no queue, and no motion object that can keep driving
 * after the OpMode ends.
 *
 * <p>That is the whole design. If a routine needs behavior this does not have, write
 * it out of these calls instead of adding a layer underneath them.
 *
 * <h2>References</h2>
 *
 *  This is the API most routines use. The conventions it assumes:
 *    - setPower / driveField take forward, strafe, turn in the range -1 to 1.
 *      setPower is robot-relative, driveField is field-relative.
 *    - Positive strafe is the robot's LEFT, matching Drivetrain.drive().
 *    - goTo / turnTo / turnBy are non-blocking: they set a target, update() steers
 *      toward it, and isBusy() reports when it is done.
 *    - Units are inches and degrees everywhere. See Pose.
 *
 *  Field-relative driving, and the rotation that makes it work:
 *    https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
 *    https://gm0.org/en/latest/docs/common-mechanisms/drivetrains/holonomic.html
 *
 *  The control law is a plain P on position with a small D on heading, tuned in
 *  DriveConfig. An integral term is deliberately absent: it converges nicely on the
 *  field you tuned it on and badly on every other one.
 */
public class Drive {

    private final Drivetrain drivetrain;
    private final Odometry odometry;
    private final DriveConfig config;

    /** Non-null exactly when we are steering toward a target. */
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

    // ------------------------------------------------------------------
    // Direct control
    // ------------------------------------------------------------------

    /**
     * Drives the wheels immediately. All three arguments are -1 to 1 and are relative
     * to the robot: forward is the direction it faces, strafe is its left, turn is
     * counter-clockwise.
     *
     * <p>Returns as soon as the power is written and remembers nothing, which is what
     * you want in TeleOp. Cancels any {@link #goTo} target.
     */
    public void setPower(double forward, double strafe, double turn) {
        cancel();
        drivetrain.drive(forward, strafe, turn);
    }

    /**
     * Same as {@link #setPower}, but the inputs are relative to the field rather than
     * the robot. Push the stick forward and the robot heads toward field +Y, so the
     * controls mean the same thing no matter which way the robot is facing.
     */
    public void driveField(double forward, double strafe, double turn) {
        double heading = Math.toRadians(odometry.getPose().getHeading());
        double sin = Math.sin(heading);
        double cos = Math.cos(heading);
        // Field-frame command -> robot-frame power. This is the INVERSE of the
        // rotation in update(), and the fact that they are not the same expression is
        // the easiest thing to get wrong here.
        //
        // Sanity check, and it is the only one that matters: at heading 0 (facing
        // field +Y) a field +Y command must come out as pure forward, and a field +X
        // command as pure strafe-right.
        setPower(
                forward * sin + strafe * cos,
                -forward * cos + strafe * sin,
                turn);
    }

    /** Stops the motors and drops any target. Also called when an OpMode exits. */
    public void stop() {
        cancel();
        drivetrain.stop();
    }

    // ------------------------------------------------------------------
    // "Go there"
    // ------------------------------------------------------------------

    /**
     * Drives to a point on the field. Arrival is judged on position only; the robot
     * keeps whatever heading it has when it gets there. If you care about the heading,
     * pass one.
     *
     * @param xInches field X, in inches
     * @param yInches field Y, in inches
     */
    public void goTo(double xInches, double yInches) {
        this.target = new Pose(xInches, yInches, odometry.getPose().getHeading());
        this.requireHeading = false;
    }

    /**
     * Drives to a point on the field and ends up facing a heading.
     *
     * <p>Non-blocking: it returns immediately, {@link #update()} does the driving, and
     * {@link #isBusy()} reports when it is done. That is the whole sequencing
     * mechanism, so a routine is just a loop that waits for {@code isBusy()} to go
     * false. A step can never be skipped and the routine can never lose track of
     * where it is.
     *
     * <p>Position and heading converge together, each correcting as fast as its own
     * error warrants. If one has to be held tightly while the other does not matter,
     * drive there first and then call {@link #turnTo}.
     *
     * <h2>Not for short moves</h2>
     *
     * <p>A target closer than {@link DriveConfig#positionToleranceInches} is treated as
     * already reached, so the robot does not move at all. With the default 2 inch
     * tolerance, {@code goTo(x + 1, y)} is a no-op that reports success. That is
     * intentional; see the tolerance comment in DriveConfig. For inch-scale nudges use
     * {@link #setPower} for a fixed time, or lower the tolerance for that step.
     *
     * @param headingDegrees field heading: 0 is along field +Y, counter-clockwise
     */
    public void goTo(double xInches, double yInches, double headingDegrees) {
        this.target = new Pose(xInches, yInches, MathUtils.wrapDegrees(headingDegrees));
        this.requireHeading = true;
    }

    /** Drives to a {@link Pose}, using its heading. */
    public void goTo(Pose pose) {
        goTo(pose.getX(), pose.getY(), pose.getHeading());
    }

    /**
     * Turns in place to a field heading. Non-blocking; wait on {@link #isBusy()}.
     * Implemented as a {@code goTo} to the current position, so it holds the spot
     * while it spins.
     */
    public void turnTo(double headingDegrees) {
        Pose pose = odometry.getPose();
        goTo(pose.getX(), pose.getY(), headingDegrees);
    }

    /**
     * Turns a number of degrees from the current heading. Negative is right, positive
     * is left. Non-blocking; wait on {@link #isBusy()}.
     */
    public void turnBy(double degrees) {
        turnTo(odometry.getPose().getHeading() + degrees);
    }

    // ------------------------------------------------------------------
    // State
    // ------------------------------------------------------------------

    /**
     * Advances odometry and steers toward the current target, if there is one.
     * Call once per loop. With no target set, it only refreshes the pose.
     */
    public void update() {
        odometry.update();
        if (target == null) {
            return;
        }

        Pose pose = odometry.getPose();
        xError = target.getX() - pose.getX();
        yError = target.getY() - pose.getY();
        headingError = MathUtils.headingError(pose.getHeading(), target.getHeading());

        // Rotate the field-frame error into the robot's frame, so that zero error
        // always commands zero and no target direction is special-cased. This is the
        // 0 deg = +Y convention on Pose, so the check is: at heading 0 a field +Y
        // error must come out as pure forward, and a field +X error as pure right.
        double heading = Math.toRadians(pose.getHeading());
        double sin = Math.sin(heading);
        double cos = Math.cos(heading);
        double forward = (xError * sin + yError * cos) * config.translationalGain;
        double strafe = (-xError * cos + yError * sin) * config.translationalGain;

        // The heading term gets a derivative term because a spinning robot overshoots
        // badly with P alone, and the IMU measures that overshoot well. The position
        // term is P only on purpose: wheel velocity is a poor, frame-ambiguous
        // signal, and a P loop there is predictable in a way a D loop is not.
        double turn = requireHeading
                ? headingError * config.headingGain
                        - odometry.getYawVelocity() * config.headingGainDamp
                : 0.0;

        turn = MathUtils.clamp(turn, -config.maxPower, config.maxPower);

        // Normalize after the gains so the diagonal of a strafing move does not exceed
        // maxPower, and the four motors never get a command they cannot deliver.
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

    /**
     * True while there is still work to do; false once a {@code goTo} has arrived or
     * the drivetrain is back under direct driver control.
     */
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

    /** Forgets the current target and hands the wheels back to the driver. */
    public void cancel() {
        target = null;
        xError = 0.0;
        yError = 0.0;
        headingError = 0.0;
    }

    // ------------------------------------------------------------------
    // Reading the robot
    // ------------------------------------------------------------------

    /** The robot's current pose, in inches and degrees. */
    public Pose getPose() {
        return odometry.getPose();
    }

    /** Tells the robot where it is, overriding what it computed. Use at auto start. */
    public void setPose(double xInches, double yInches, double headingDegrees) {
        odometry.setPose(new Pose(xInches, yInches, headingDegrees));
    }

    /** Tells the robot where it is, overriding what it computed. */
    public void setPose(Pose pose) {
        odometry.setPose(pose);
    }

    /**
     * Zeroes the wheel encoders and puts the pose back at the origin. TeleOp-only
     * convenience, for when you push the robot back to a known spot and want a clean
     * reference. Never call this during a match; it throws away the position the
     * odometry has earned.
     */
    public void resetEncodersAndPose() {
        drivetrain.resetEncoders();
        odometry.setPose(new Pose(0, 0, 0));
    }

    /** Distance to the current target in inches; 0 when there is no target. */
    public double getPositionError() {
        return Math.sqrt(xError * xError + yError * yError);
    }

    /** Heading error to the current target in degrees; 0 when there is no target. */
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
