package org.firstinspires.ftc.teamcode.util;

/**
 * Pure math helpers. No hardware, no OpMode state, no side effects.
 *
 * <p>Everything here is deterministic and easy to unit test on a plain JVM, which
 * matters because these are the functions you will be tuning mid-competition.
 *
 * <h2>References</h2>
 *
 *  Pure math: angles, clamping, normalization, dead zones. No SDK types, so this
 *  is the one file here that can be unit tested without a robot.
 */
public final class MathUtils {

    private MathUtils() {
    }

    /** Smallest change in heading that is considered "there" when rotating. */
    public static final double DEFAULT_TOLERANCE_DEGREES = 1.5;

    /** Constrains {@code value} to the inclusive range [min, max]. */
    public static double clamp(double value, double min, double max) {
        if (value < min) {
            return min;
        }
        return value > max ? max : value;
    }

    /** Constrains {@code value} to [0, max]. */
    public static double clampToMax(double value, double max) {
        return clamp(value, 0.0, max);
    }

    /** Returns -1, 0, or 1 according to the sign of {@code value}. */
    public static double sign(double value) {
        if (value > 0) {
            return 1.0;
        }
        return value < 0 ? -1.0 : 0.0;
    }

    /**
     * Wraps an angle into the range [-180, 180) degrees.
     *
     * <p>Use this instead of {@code angle % 180} style tricks, which stay wrong for
     * negative inputs and are the usual source of "the robot turned the wrong way".
     */
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

    /**
     * Returns the shortest signed rotation that takes the robot from {@code current}
     * to {@code target}, in degrees. Always in (-180, 180].
     *
     * <p>This is the function you want for every "turn toward" decision, because it
     * always takes the shorter way around and never spins the long way for 2 degrees.
     */
    public static double headingError(double current, double target) {
        return wrapDegrees(target - current);
    }

    /**
     * Scales a set of motor powers down so the largest magnitude is at most
     * {@code max}, leaving the ratios between them untouched.
     *
     * <p>This is how mecanum keeps a diagonal request from commanding 1.4 to a motor
     * and slamming into the motor's current limit. Passing a larger value than 1.0 is
     * allowed on purpose: some FTC drivetrains run at partial derating on purpose.
     */
    public static void normalize(double[] powers, double max) {
        double largest = 0.0;
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

    /** True when {@code a} and {@code b} are within {@code tolerance} of each other. */
    public static boolean nearlyEqual(double a, double b, double tolerance) {
        return Math.abs(a - b) <= tolerance;
    }

    /**
     * Applies a dead zone to a joystick axis and rescales the remaining range so the
     * output still reaches a full 0 to 1. Without the rescale, ignoring the first
     * few percent of stick travel makes the robot permanently slightly slower.
     */
    public static double applyDeadZone(double value, double deadZone) {
        double magnitude = Math.abs(value);
        if (magnitude <= deadZone) {
            return 0.0;
        }
        double sign = sign(value);
        return sign * (magnitude - deadZone) / (1.0 - deadZone);
    }

    /** Squares a value while preserving its sign. Sharpens small stick inputs. */
    public static double signedSquare(double value) {
        return value * Math.abs(value);
    }
}
