package org.firstinspires.ftc.teamcode.util;

/**
 * A single PIDF controller, used for anything that needs to settle on a value:
 * driving to a heading, driving to a coordinate, holding a flywheel speed.
 *
 * <p>Written in FTC-native units (inches, degrees, inches/second, degrees/second)
 * so gains can be reasoned about directly. A gain is roughly
 * "output per unit of error at the start of a move".
 *
 * <p>Pure math: no hardware, no time source. {@link #update(double error, double dt)}
 * takes the elapsed time as a parameter, which is what makes it testable.
 */
public class Pidf {

    private final double kP;
    private final double kI;
    private final double kD;
    private final double kF;

    private double integral = 0.0;
    private double lastError = 0.0;
    private boolean hasLastError = false;

    /** Velocity in output units per second that {@code kF} multiplies. Set 0 if unused. */
    private final double feedForwardVelocity;

    public Pidf(double kP, double kI, double kD, double kF) {
        this(kP, kI, kD, kF, 0.0);
    }

    public Pidf(double kP, double kI, double kD, double kF, double feedForwardVelocity) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.feedForwardVelocity = feedForwardVelocity;
    }

    /** Clears integral windup and derivative history. Call this at the start of every move. */
    public void reset() {
        integral = 0.0;
        lastError = 0.0;
        hasLastError = false;
    }

    /**
     * Computes the command for the current error.
     *
     * @param error how far the current value is from the target, already signed
     * @param dt    seconds since the previous update
     */
    public double update(double error, double dt) {
        double output = kP * error + kF * feedForwardVelocity;

        // Integrate only when the dt is sane, so a paused OpMode or a dropped frame
        // does not dump a huge chunk of integral into the next update.
        if (dt > 0.0 && dt < 0.5) {
            integral += error * dt;
            output += kI * integral;
        }

        if (hasLastError && dt > 0.0) {
            output += kD * (error - lastError) / dt;
        }

        lastError = error;
        hasLastError = true;
        return output;
    }

    /**
     * Stops accumulating integral once the controller is close to the target.
     *
     * <p>Without this, a move that overshoots and comes back keeps unwinding a large
     * integral and oscillates instead of settling.
     */
    public double updateWithIntegralCutoff(double error, double dt, double integralCutoffDistance) {
        if (Math.abs(error) > integralCutoffDistance) {
            return update(error, dt);
        }
        // Freeze the integral but still run P and D.
        double frozenIntegral = integral;
        double result = update(error, dt);
        integral = frozenIntegral;
        return result;
    }
}
