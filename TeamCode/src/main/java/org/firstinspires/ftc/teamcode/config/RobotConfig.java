package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drivetrain.Pose;
import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.Length;
import org.firstinspires.ftc.teamcode.util.Units;

// hardware names and measurements for this robot, the only file that holds either
// https://ftc-docs.firstinspires.org/en/latest/ftc_sdk/overview/index.html
public class RobotConfig {
    // units used by every measurement in this file and by drive coordinates
    // the robot stores inches and degrees internally, these are the authoring units
    public DistanceUnit distanceUnit = DistanceUnit.CM;
    public AngleUnit angleUnit = AngleUnit.DEGREES;

    // must match driver station exactly, case included
    public String frontLeft = "FL";
    public String frontRight = "FR";
    public String backLeft = "BL";
    public String backRight = "BR";

    // motor directions
    // set them all to forward, run teleop, flip any that move the wrong way
    public DcMotor.Direction frontLeftDirection = DcMotor.Direction.FORWARD;
    public DcMotor.Direction frontRightDirection = DcMotor.Direction.REVERSE;
    public DcMotor.Direction backLeftDirection = DcMotor.Direction.REVERSE;
    public DcMotor.Direction backRightDirection = DcMotor.Direction.FORWARD;

    // coast slides, brake stops crisper and holds position in auto
    // brake is the safer default for auto, it holds position
    public ZeroPowerBehavior zeroPowerBehavior = ZeroPowerBehavior.BRAKE;

    /// measure these, do not guess
    // wheelRadius is outer edge to axle, trackWidth face to face
    // centerTrackWidth is center to center, wheelBase front to back
    public double wheelRadius = 2.0;
    public double trackWidth = 15.0;
    public double centerTrackWidth = 13.0;
    public double wheelBase = 13.0;

    // gearing and encoder
    // gearRatio is reduction from motor shaft to wheel
    public double gearRatio = 1.0;
    public double motorTicksPerRevolution = 28.0;

    // driver feel
    public double deadZone = 0.05;
    public double maxPower = 1.0;

    // joystick directions
    public StickMotion leftX = StickMotion.Strafe;
    public StickMotion leftY = StickMotion.Move;
    public StickMotion rightX = StickMotion.Turn;
    public StickMotion rightY = StickMotion.None;

    // flips a stick motion
    public boolean invertMove = true;
    public boolean invertTurn = false;
    public boolean invertStrafe = false;

    // scales every auto and goTo command
    public double autoPowerScale = 1.0;

    // command = error * gain
    public double translationalGain = 0.09;
    public double headingGain = 0.045;
    public double headingGainDamp = 0.004;

    // a goTo is done when it is inside every tolerance it needs
    // 2 is loose on purpose, pinpoint drifts more than that over a full match
    public double positionTolerance = 2.0;
    public double headingTolerance = 2.0;

    // where the robot is when the opmode starts, in the units above
    // set this to the real starting tile so field-relative means the real field
    public double startX = 0.0;
    public double startY = 0.0;
    public double startHeading = 0.0;

    // odometry hardware
    // false falls back to default imu
    public boolean usePinpoint = true;
    public String pinpoint = "pinpoint";

    public GoBildaPinpointDriver.GoBildaOdometryPods odometryPods =
        GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    public GoBildaPinpointDriver.EncoderDirection xEncoderDirection =
        GoBildaPinpointDriver.EncoderDirection.REVERSED;
    public GoBildaPinpointDriver.EncoderDirection yEncoderDirection =
        GoBildaPinpointDriver.EncoderDirection.FORWARD;

    // pod offsets from the robot center, signed
    // podX is forward pod's left positive, podY is strafe pod's forward positive
    public double podXOffset = -3.75;
    public double podYOffset = 0.75;

    /// unit helpers
    // read a number in the configured unit
    public Length length(double value) {
        return Length.of(value, distanceUnit);
    }

    public Angle angle(double value) {
        return Angle.of(value, angleUnit);
    }

    // convert a configured number to the internal unit
    public double toInches(double value) {
        return Units.toInches(value, distanceUnit);
    }

    public double toDegrees(double value) {
        return Units.toDegrees(value, angleUnit);
    }

    // convert an internal number to the configured unit
    public double fromInches(double inches) {
        return Units.fromInches(inches, distanceUnit);
    }

    public double fromDegrees(double degrees) {
        return Units.fromDegrees(degrees, angleUnit);
    }

    // starting pose in field coordinates, already converted to inches and degrees
    public Pose startPose() {
        return new Pose(startX, startY, startHeading, distanceUnit, angleUnit);
    }

    // shared instance everything else reads
    public static final RobotConfig ROBOT = new RobotConfig();
}
