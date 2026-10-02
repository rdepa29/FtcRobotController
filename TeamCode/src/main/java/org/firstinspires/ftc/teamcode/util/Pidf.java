package org.firstinspires.ftc.teamcode.util;

// textbook pidf in FTC-native units so gains can be reasoned about directly
// nothing calls this yet, it is here for the lifters and flywheels
public class Pidf {

    private final double kP;
    private final double kI;
    private final double kD;
    private final double kF;

    private double integral = 0.0;
    private double lastError = 0.0;
    private boolean hasLastError = false;

    // output units per second that kF multiplies, 0 if unused
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

    // clears integral windup and derivative history, call at the start of every
    // move
    public void reset() {
        integral = 0.0;
        lastError = 0.0;
        hasLastError = false;
    }

    // computes the command for the current error, error is already signed
    // dt is seconds since the previous update
    public double update(double error, double dt) {
        double output = kP * error + kF * feedForwardVelocity;

        // integrate only when dt is sane, so a paused opmode or a dropped frame
        // does not dump a huge integral into the next update
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

    // stops accumulating integral once close to the target
    // without this an overshoot keeps unwinding a large integral and oscillates
    public double updateWithIntegralCutoff(double error, double dt, double integralCutoffDistance) {
        if (Math.abs(error) > integralCutoffDistance) {
            return update(error, dt);
        }
        // freeze the integral but still run p and d
        double frozenIntegral = integral;
        double result = update(error, dt);
        integral = frozenIntegral;
        return result;
    }
}
