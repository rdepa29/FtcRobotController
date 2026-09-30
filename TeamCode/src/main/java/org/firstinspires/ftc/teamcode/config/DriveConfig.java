package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

// hardware names and measurements for this robot, the only file that should
// hold either
//
// https://ftc-docs.firstinspires.org/en/latest/ftc_sdk/overview/index.html
public class DriveConfig {

    // must match hardware.xml exactly, case included
    public String frontLeft = "FL";
    public String frontRight = "FR";
    public String backLeft = "BL";
    public String backRight = "BR";

    // motor directions
    public DcMotor.Direction frontLeftDirection = DcMotor.Direction.REVERSE;
    public DcMotor.Direction frontRightDirection = DcMotor.Direction.FORWARD;
    public DcMotor.Direction backLeftDirection = DcMotor.Direction.REVERSE;
    public DcMotor.Direction backRightDirection = DcMotor.Direction.FORWARD;

    // coast slides, brake stops crisper and holds position in auto
    public ZeroPowerBehavior zeroPowerBehavior = ZeroPowerBehavior.BRAKE;

    // set true if strafing the wrong way, fix it here and not in the mixing
    public boolean mirrorStrafe = false;

    // measure these, do not guess
    public double centerTrackWidthInches = 13.0;
    public double wheelBaseInches = 13.0;

    // gearRatio is reduction from motor shaft to wheel, encoder odometry is
    // meaningless if it is wrong
    public double gearRatio = 1.0;
    public double motorTicksPerRevolution = 28.0;

    // driver feel
    public double deadZone = 0.05;
    public double maxPower = 1.0;

    // scales every auto and goTo command
    public double autoPowerScale = 1.0;

    // p gains, command = error * gain
    public double podYOffsetInches = 0.75;

    // shared instance everything else reads
    public static final DriveConfig ROBOT = new DriveConfig();
}
