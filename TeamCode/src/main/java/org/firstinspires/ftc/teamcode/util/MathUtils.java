package org.firstinspires.ftc.teamcode.util;

// angle wrapping, clamping, normalization and dead zones, no SDK types so this
// is the one file here that unit tests on a plain jvm
public final class MathUtils {

    private MathUtils() {
    }

    // smallest heading change that counts as there when rotating
    public static final double DEFAULT_TOLERANCE_DEGREES = 1.5;

    // value clamped to the inclusive range [min,max]
    public static double clamp(double value, double min, double max) {
        if (value < min) {
            return min;
        }
        return value > max ? max : value;
    }

    // value clamped to [0,max]
    public static double clampToMax(double value, double max) {
        return clamp(value, 0.0, max);
    }

    // -1, 0 or 1 according to the sign of value
    public static double sign(double value) {
        if (value > 0) {
            return 1.0;
        }
        return value < 0 ? -1.0 : 0.0;
    }

    // wraps into [-180,180), use this instead of % tricks which stay wrong on
    // negative input and are why a robot turns the wrong way
    public static double wrapDegrees(double degrees) {
        double wrapped = degrees % 360.0;
        if (wrapped >= 180.0) {
            wrapped -= 360.0;
        }
        if (wrapped < -180.0) {
            wrapped += 360.0;
        }
        return wrapped;
    }

    // shortest signed rotation from current to target, always in (-180,180]
        return wrapDegrees(target - current);
    }

    // scales powers down so the largest is at most max, ratios untouched
        for (double power : powers) {
            double magnitude = Math.abs(power);
            if (magnitude > largest) {
                largest = magnitude;
            }
        }
        if (largest <= max || largest == 0.0) {
            return;
        }
        double scale = max / largest;
        for (int i = 0; i < powers.length; i++) {
            powers[i] *= scale;
        }
    }

    // true when a and b are within tolerance of each other
    public static boolean nearlyEqual(double a, double b, double tolerance) {
        return Math.abs(a - b) <= tolerance;
    }

    // dead zone with a rescale so the output still reaches a full 1, without the
    // rescale the robot stays permanently slightly slow
    public static double applyDeadZone(double value, double deadZone) {
        double magnitude = Math.abs(value);
        if (magnitude <= deadZone) {
            return 0.0;
        }
        double sign = sign(value);
        return sign * (magnitude - deadZone) / (1.0 - deadZone);
    }

    // squared but keeps the sign, sharpens small stick inputs
    public static double signedSquare(double value) {
        return value * Math.abs(value);
    }
}
