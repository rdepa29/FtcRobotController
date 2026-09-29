package org.firstinspires.ftc.teamcode.drivetrain;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.MathUtils;

/**
 * A robot position on the field, always in inches and degrees.
 *
 * <h2>Why this class exists</h2>
 *
 * <p>The SDK's {@link Pose2D} stores the units alongside the numbers, which means every
 * read has to say which unit you want:
 *
 * <pre>
 *   pose2d.getX(INCH)
 * </pre>
 *
 * <p>That is a compile error waiting to be avoided, and the one way it goes wrong is
 * quietly. If one object is in millimeters and another is in inches, subtracting them
 * still compiles, still runs, and gives you a number that is off by a factor of 25.4.
 * That failure is invisible on the Driver Station because a wrong position is still a
 * number.
 *
 * <p>{@code Pose} has no unit fields at all. The units are fixed, the class is
 * immutable, and a call site physically cannot disagree about them.
 *
 * <h2>Frame convention</h2>
 *
 * <p>(0, 0) is one corner of the field and (144, 144) is the opposite corner, in
 * inches. Heading 0 degrees means facing along the +Y axis, positive headings turn
 * counter-clockwise. See {@link org.firstinspires.ftc.teamcode.config.FieldConfig}
 * for which physical corner that is, and confirm it against the field CAD.
 *
 * <h2>References</h2>
 *
 *  The type being wrapped is Pose2D, which carries its units in the fields, so every
 *  read has to name one:
 *    https://javadoc.io/doc/org.firstinspires.ftc/RobotCore/12.0.0/index.html
 *
 *  Coordinate convention used throughout this codebase, and the one FTC uses:
 *    - Position is in inches. The field is 144 x 144.
 *    - (0,0) is one corner of the field. Which corner is still a MEASURE item.
 *    - Heading 0 means facing along field +Y. Positive turns counter-clockwise.
 *    - Rotation is therefore NOT the usual math convention. Rotating a field vector
 *      into the robot frame is [[sin, cos], [-cos, sin]], the transpose of the
 *      robot-to-field matrix. Getting this backwards still compiles and still makes
 *      plausible telemetry, so it is worth testing numerically; both directions are
 *      written out in Drive.
 */
public final class Pose {

    public static final DistanceUnit DISTANCE_UNIT = DistanceUnit.INCH;
    public static final AngleUnit ANGLE_UNIT = AngleUnit.DEGREES;

    private final double x;
    private final double y;
    private final double heading;

    public Pose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = MathUtils.wrapDegrees(heading);
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    /** Heading in degrees, already wrapped into [-180, 180). */
    public double getHeading() {
        return heading;
    }

    /** Straight-line distance from this pose to {@code other}, in inches. */
    public double distanceTo(Pose other) {
        double dx = other.x - x;
        double dy = other.y - y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    /**
     * Angle from this pose toward {@code other}, in degrees, where 0 means
     * straight along +Y. This is the angle you feed a turn controller to point the
     * robot at a target.
     */
    public double bearingTo(Pose other) {
        return MathUtils.wrapDegrees(Math.toDegrees(Math.atan2(other.x - x, other.y - y)));
    }

    /** Shortest signed rotation from this heading to {@code other}'s heading. */
    public double headingErrorTo(Pose other) {
        return MathUtils.headingError(heading, other.heading);
    }

    /** This pose moved by (dx, dy) in the field frame, with heading unchanged. */
    public Pose translate(double dx, double dy) {
        return new Pose(x + dx, y + dy, heading);
    }

    /** This pose moved along its own heading by {@code forwardInches}. */
    public Pose projectForward(double forwardInches) {
        double radians = Math.toRadians(heading);
        return translate(forwardInches * Math.sin(radians), forwardInches * Math.cos(radians));
    }

    /** This pose moved to the right of its own heading by {@code rightInches}. */
    public Pose projectRight(double rightInches) {
        double radians = Math.toRadians(heading);
        return translate(rightInches * Math.cos(radians), -rightInches * Math.sin(radians));
    }

    /** Converts to the SDK type. The reverse direction, {@link #fromPose2D}, is safe. */
    public Pose2D toPose2D() {
        return new Pose2D(DISTANCE_UNIT, x, y, ANGLE_UNIT, heading);
    }

    /**
     * Converts from a {@link Pose2D}, reading out its values in our units regardless
     * of what units the source happened to be built with.
     */
    public static Pose fromPose2D(Pose2D pose2D) {
        return new Pose(
                pose2D.getX(DISTANCE_UNIT),
                pose2D.getY(DISTANCE_UNIT),
                pose2D.getHeading(ANGLE_UNIT));
    }

    public Pose2D toPose2D(DistanceUnit distanceUnit, AngleUnit angleUnit) {
        return new Pose2D(
                distanceUnit,
                distanceUnit.fromUnit(DISTANCE_UNIT, x),
                distanceUnit.fromUnit(DISTANCE_UNIT, y),
                angleUnit,
                angleUnit.fromUnit(ANGLE_UNIT, heading));
    }

    @Override
    public String toString() {
        return String.format("(%.1f, %.1f, %.1f deg)", x, y, heading);
    }

    @Override
    public boolean equals(Object other) {
        if (this == other) {
            return true;
        }
        if (!(other instanceof Pose)) {
            return false;
        }
        Pose that = (Pose) other;
        return MathUtils.nearlyEqual(x, that.x, 1e-6)
                && MathUtils.nearlyEqual(y, that.y, 1e-6)
                && MathUtils.nearlyEqual(heading, that.heading, 1e-6);
    }

    @Override
    public int hashCode() {
        return (int) ((x * 31 + y) * 31 + heading);
    }
}
