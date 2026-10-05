package org.firstinspires.ftc.teamcode.config;

/// what a stick axis is wired to do
// the driver decides, DriveConfig assigns, DriverInput sums
public enum StickMotion {
    /// ignored, reads as zero
    None,
    /// forward and back
    Move,
    /// left and right
    Strafe,
    /// CCW and CW
    Turn
}