package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// an angle with a unit attached, stored as degrees
public final class Angle {
    private final double degrees;

    private Angle(double degrees) {
        this.degrees = degrees;
    }

    public static Angle of(double value, AngleUnit unit) {
        return new Angle(Units.toDegrees(value, unit));
    }

    public static Angle degrees(double value) {
        return new Angle(value);
    }

    public static Angle radians(double value) {
        return of(value, AngleUnit.RADIANS);
    }

    public double to(AngleUnit unit) {
        return Units.fromDegrees(degrees, unit);
    }

    public double toDegrees() {
        return degrees;
    }

    public double toDeg() {
        return degrees;
    }

    public double toRadians() {
        return to(AngleUnit.RADIANS);
    }

    public double toRad() {
        return toRadians();
    }

    public Angle plus(Angle other) {
        return new Angle(degrees + other.degrees);
    }

    public Angle minus(Angle other) {
        return new Angle(degrees - other.degrees);
    }

    @Override
    public String toString() {
        return String.format("%.2f deg", degrees);
    }

    @Override
    public boolean equals(Object other) {
        if (this == other) {
            return true;
        }
        if (!(other instanceof Angle)) {
            return false;
        }
        return MathUtils.nearlyEqual(degrees, ((Angle) other).degrees, 1e-9);
    }

    @Override
    public int hashCode() {
        return (int) (degrees * 1000000.0);
    }
}
