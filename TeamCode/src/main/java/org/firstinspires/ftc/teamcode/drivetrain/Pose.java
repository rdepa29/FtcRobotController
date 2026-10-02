package org.firstinspires.ftc.teamcode.drivetrain;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.MathUtils;

// a position in inches and degrees, units you cannot argue about
// heading 0 faces field +Y and positive is CCW
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

    // degrees, already wrapped into [-180,180)
    public double getHeading() {
        return heading;
    }

    // straight line distance to other, in inches
    public double distanceTo(Pose other) {
        double dx = other.x - x;
        double dy = other.y - y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    // bearing to other in degrees where 0 is straight along +Y, this is what a
    // turn controller wants to point at a target
    public double bearingTo(Pose other) {
        return MathUtils.wrapDegrees(Math.toDegrees(Math.atan2(other.x - x, other.y - y)));
    }

    // shortest signed rotation to other's heading
    public double headingErrorTo(Pose other) {
        return MathUtils.headingError(heading, other.heading);
    }

    // moved by (dx,dy) in the field frame, heading unchanged
    public Pose translate(double dx, double dy) {
        return new Pose(x + dx, y + dy, heading);
    }

    // moved forward along our own heading
    public Pose projectForward(double forwardInches) {
        double radians = Math.toRadians(heading);
        return translate(forwardInches * Math.sin(radians), forwardInches * Math.cos(radians));
    }

    // moved right of our own heading
    public Pose projectRight(double rightInches) {
        double radians = Math.toRadians(heading);
        return translate(rightInches * Math.cos(radians), -rightInches * Math.sin(radians));
    }

    // to the SDK type, fromPose2D is the safe way back
    public Pose2D toPose2D() {
        return new Pose2D(DISTANCE_UNIT, x, y, ANGLE_UNIT, heading);
    }

    // from a Pose2D, read in our units whatever units it was built with
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
