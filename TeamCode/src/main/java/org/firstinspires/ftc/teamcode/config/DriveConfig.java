package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

// hardware names and measurements for this robot, the only file that holds either
// https://ftc-docs.firstinspires.org/en/latest/ftc_sdk/overview/index.html
public class DriveConfig {

    // must match driver station exactly, case included
    public String frontLeft = "FL";
    public String frontRight = "FR";
    public String backLeft = "BL";
    public String backRight = "BR";

    // motor directions
    // set them all to forward, run teleop, flip any that move the wrong way
    public DcMotor.Direction frontLeftDirection = DcMotor.Direction.REVERSE;
    public DcMotor.Direction frontRightDirection = DcMotor.Direction.FORWARD;
    public DcMotor.Direction backLeftDirection = DcMotor.Direction.REVERSE;
    public DcMotor.Direction backRightDirection = DcMotor.Direction.FORWARD;

    // coast slides, brake stops crisper and holds position in auto
    // brake is the safer default for auto, it holds position
    public ZeroPowerBehavior zeroPowerBehavior = ZeroPowerBehavior.BRAKE;

    /// mecanum roller arrangement
    // set true if strafe right moves the robot left
    // fix it here and not in the mixing
    public boolean mirrorStrafe = false;

    /// measure these, do not guess
    // wheelRadius outer edge to axle, trackWidth face to face
    // centerTrackWidth center to center, wheelBase front to back
    public double wheelRadiusInches = 2.0;
    public double trackWidthInches = 15.0;
    public double centerTrackWidthInches = 13.0;
    public double wheelBaseInches = 13.0;

    // gearing and encoder
    // gearRatio is reduction from motor shaft to wheel
    public double gearRatio = 1.0;
    public double motorTicksPerRevolution = 28.0;

    // driver feel
    public double deadZone = 0.05;
    public double maxPower = 1.0;

    // scales every auto and goTo command
    public double autoPowerScale = 1.0;

    // closed loop gains, command = error * gain
    public double translationalGain = 0.09;
    public double headingGain = 0.045;
    public double headingGainDamp = 0.004;

    // a goTo is done when it is inside every tolerance it needs
    // 2in is loose on purpose, pinpoint drifts more than that over a full match
    public double positionToleranceInches = 2.0;
    public double headingToleranceDegrees = 2.0;

    // odometry hardware
    // set false to use the encoder fallback even with a pinpoint wired up
    public boolean usePinpoint = true;
    public String pinpoint = "pinpoint";

    public GoBildaPinpointDriver.GoBildaOdometryPods odometryPods =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    public GoBildaPinpointDriver.EncoderDirection xEncoderDirection =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public GoBildaPinpointDriver.EncoderDirection yEncoderDirection =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;

    // pod offsets from the robot center, signed, in inches
    // podX forward pod left positive, podY strafe pod forward positive
    public double podXOffsetInches = -3.75;
    public double podYOffsetInches = 0.75;

    // shared instance everything else reads
    public static final DriveConfig ROBOT = new DriveConfig();
}
