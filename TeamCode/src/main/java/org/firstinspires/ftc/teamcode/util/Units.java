package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// converts between the SDK units, the codebase stores inches and degrees
public final class Units {
    private Units() {
    }

    public static double toInches(double value, DistanceUnit unit) {
        return DistanceUnit.INCH.fromUnit(unit, value);
    }

    public static double fromInches(double inches, DistanceUnit unit) {
        return unit.fromUnit(DistanceUnit.INCH, inches);
    }

    public static double toDegrees(double value, AngleUnit unit) {
        return AngleUnit.DEGREES.fromUnit(unit, value);
    }

    public static double fromDegrees(double degrees, AngleUnit unit) {
        return unit.fromUnit(AngleUnit.DEGREES, degrees);
    }
}
