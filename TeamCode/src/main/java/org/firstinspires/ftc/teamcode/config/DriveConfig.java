package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

/**
 * Everything about the drivetrain that is a fact about the physical robot.
 *
 * <p>This is the file the team edits. Nothing else in the codebase should contain a
 * hardware name, a motor direction, or a measurement taken with a tape measure.
 *
 * <p>Fields are intentionally public and mutable rather than hidden behind a builder,
 * because the fastest way to change a robot is to change one line here and re-push.
 * Nothing mutates these after startup.
 */
public class DriveConfig {

    // ------------------------------------------------------------------
    // Hardware names. These must match the names in your hardware.xml.
    // ------------------------------------------------------------------
    public String frontLeft = "FL";
    public String frontRight = "FR";
    public String backLeft = "BL";
    public String backRight = "BR";

    // ------------------------------------------------------------------
    // Motor directions.
    //
    // Set every one of these to FORWARD, push, and let the driver hold the robot
    // while you run a TeleOp. Flip the ones that move the wrong way. Flipping a
    // direction here is the correct fix; editing a sign inside a motion class is not.
    // ------------------------------------------------------------------
    public DcMotor.Direction frontLeftDirection = DcMotor.Direction.REVERSE;
    public DcMotor.Direction frontRightDirection = DcMotor.Direction.FORWARD;
    public DcMotor.Direction backLeftDirection = DcMotor.Direction.REVERSE;
    public DcMotor.Direction backRightDirection = DcMotor.Direction.FORWARD;

    // ------------------------------------------------------------------
    // What the motors do when commanded to zero.
    //
    // For a drivetrain, COAST gives you a looser, faster robot that slides a little
    // on a stop. BRAKE gives you crisp stops and repeatable autonomous endpoints.
    // BRAKE is the safer default for competitive autonomous.
    // ------------------------------------------------------------------
    public ZeroPowerBehavior zeroPowerBehavior = ZeroPowerBehavior.BRAKE;

    // ------------------------------------------------------------------
    // Mecanum roller arrangement.
    //
    // Mecanum wheels come in two mirror-image roller layouts and you only find out
    // which one you have by pushing the robot sideways. If "strafe right" moves the
    // robot left, flip this to true. Do not edit the mixing formula to fix it.
    // Ignored by DifferentialDrive.
    // ------------------------------------------------------------------
    public boolean mirrorStrafe = false;

    // ------------------------------------------------------------------
    // Physical measurements. Measure these; do not guess them.
    //
    //   wheelRadius       outer edge of the wheel to its axle
    //   trackWidth        outside face of the left wheels to outside face of the right
    //   centerTrackWidth  left wheel center to right wheel center
    //   wheelBase         front wheel center to back wheel center
    // ------------------------------------------------------------------
    public double wheelRadiusInches = 2.0;
    public double trackWidthInches = 15.0;
    public double centerTrackWidthInches = 13.0;
    public double wheelBaseInches = 13.0;

    // ------------------------------------------------------------------
    // Motor gearing and encoder.
    //
    // gearRatio is the reduction between the motor shaft and the wheel, so 1.0 for
    // direct drive and 3.0 for a 3:1 belt. It must be correct before the
    // encoder-based odometry fallback means anything.
    // ------------------------------------------------------------------
    public double gearRatio = 1.0;
    public double motorTicksPerRevolution = 28.0;

    // ------------------------------------------------------------------
    // Driver feel.
    // ------------------------------------------------------------------
    public double deadZone = 0.05;
    public double maxPower = 1.0;

    /** Autonomous and motion commands are multiplied by this before running. */
    public double autoPowerScale = 1.0;

    // ------------------------------------------------------------------
    // Closed-loop gains.
    //
    // These are P gains: command = error * gain. There is deliberately no I and no D
    // on position. A plain P converges on nearly every drive goal, and an integral
    // term tuned on one field is a liability on the next one. If the robot overshoots
    // badly at speed, the fix is a lower translationalGain, not a new term.
    //
    // Heading is P plus a small D, because a spinning robot overshoots badly with P
    // alone and the IMU measures that overshoot well.
    //
    // Tune order: raise translationalGain until it reaches the point without
    // spiraling, then lower it until the overshoot is gone. Then do the same with
    // headingGain. Whichever one you are tuning, watch the other on telemetry, since
    // they fight each other.
    // ------------------------------------------------------------------
    public double translationalGain = 0.09;
    public double headingGain = 0.045;
    public double headingGainDamp = 0.004;

    // ------------------------------------------------------------------
    // Tolerances. A "goTo" is finished when it is inside all of the ones it requires.
    //
    // The default 2.0 inch position tolerance is deliberately loose. Pinpoint and the
    // encoder fallback are both worse than that over a full match, so a tight tolerance
    // just makes autonomous stall trying to reach a position it cannot measure.
    // Tighten it only where the task actually needs the accuracy.
    // ------------------------------------------------------------------
    public double positionToleranceInches = 2.0;
    public double headingToleranceDegrees = 2.0;

    // ------------------------------------------------------------------
    // Odometry hardware.
    // ------------------------------------------------------------------

    /** Set false to force the encoder-based fallback even if a Pinpoint is present. */
    public boolean usePinpoint = true;
    public String pinpoint = "pinpoint";

    public GoBildaPinpointDriver.GoBildaOdometryPods odometryPods =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    public GoBildaPinpointDriver.EncoderDirection xEncoderDirection =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public GoBildaPinpointDriver.EncoderDirection yEncoderDirection =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;

    /**
     * Position of the two odometry pods relative to the point the robot tracks,
     * which should be the robot's center. Signed, in inches.
     *
     *   podX  how far the forward (X) pod is from center, left positive
     *   podY  how far the strafe (Y) pod is from center, forward positive
     */
    public double podXOffsetInches = -3.75;
    public double podYOffsetInches = 0.75;

    /** The shared instance the rest of the code reads. */
    public static final DriveConfig ROBOT = new DriveConfig();
}
