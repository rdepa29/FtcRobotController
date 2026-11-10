package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// a distance with a unit attached, stored as inches
public final class Length {
    private final double inches;

    private Length(double inches) {
        this.inches = inches;
    }

    public static Length of(double value, DistanceUnit unit) {
        return new Length(Units.toInches(value, unit));
    }

    public static Length inches(double value) {
        return new Length(value);
    }

    public static Length centimeters(double value) {
        return of(value, DistanceUnit.CM);
    }

    public static Length millimeters(double value) {
        return of(value, DistanceUnit.MM);
    }

    public static Length meters(double value) {
        return of(value, DistanceUnit.METER);
    }

    public double to(DistanceUnit unit) {
        return Units.fromInches(inches, unit);
    }

    public double toInches() {
        return inches;
    }

    public double toCentimeters() {
        return to(DistanceUnit.CM);
    }

    public double toCm() {
        return toCentimeters();
    }

    public double toMillimeters() {
        return to(DistanceUnit.MM);
    }

    public double toMm() {
        return toMillimeters();
    }

    public double toMeters() {
        return to(DistanceUnit.METER);
    }

    public double toM() {
        return toMeters();
    }

    public Length plus(Length other) {
        return new Length(inches + other.inches);
    }

    public Length minus(Length other) {
        return new Length(inches - other.inches);
    }

    @Override
    public String toString() {
        return String.format("%.2f in", inches);
    }

    @Override
    public boolean equals(Object other) {
        if (this == other) {
            return true;
        }
        if (!(other instanceof Length)) {
            return false;
        }
        return MathUtils.nearlyEqual(inches, ((Length) other).inches, 1e-9);
    }

    @Override
    public int hashCode() {
        return (int) (inches * 1000000.0);
    }
}
