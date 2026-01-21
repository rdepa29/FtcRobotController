package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class OpModeHelper {
    /// Declarations
    // Start Runtime Debugger
    ElapsedTime runtime = new ElapsedTime();

    // Telemetry
    Telemetry telemetry;

    // Declare Drive Train Motors
    DcMotor frontLeftDrive = null;
    DcMotor backLeftDrive = null;
    DcMotor frontRightDrive = null;
    DcMotor backRightDrive = null;

    // Flywheels and Launcher
    DcMotor flywheelMotor1 = null;
    DcMotor flywheelMotor2 = null;

    DcMotor intakeMotor1 = null;
    DcMotor intakeMotor2 = null;

    // goBilda Pinpoint Odometry Computer
    //GoBildaPinpointDriver pinpoint = null;
    double xPos = 0.0;
    double yPos = 0.0;
    double heading = 0.0;
    double speedMod = 1.0;
    double autoSpeedMod = 0.2;

    // Other
    boolean canMove = true;

    public OpModeHelper(
            /// Hardware Names
            // Drive Train Motors
            String FL_NAME, String FR_NAME, String BL_NAME, String BR_NAME,
            // Shooting Mechanism Motors
            String FW1_NAME, String FW2_NAME, String IN1_NAME, String IN2_NAME,
            // Odometry Computer
            String PINPOINT_NAME,
            /// Hardware Values
            // Mapping
            HardwareMap hardwareMap,
            // Units
            DistanceUnit distanceUnit,
            AngleUnit angleUnit,
            double OffsetX,
            double OffsetY,
            // Drive Train Directions
            DcMotor.Direction FLDir, DcMotor.Direction BLDir, DcMotor.Direction FRDir, DcMotor.Direction BRDir,
            // Shooting Mechanism Directions
            DcMotor.Direction FW1Dir, DcMotor.Direction FW2Dir, DcMotor.Direction IN1Dir, DcMotor.Direction IN2Dir,
            // Odometry Computer
            GoBildaPinpointDriver.GoBildaOdometryPods OdometryPodType,
            GoBildaPinpointDriver.EncoderDirection EncoderDirectionX, GoBildaPinpointDriver.EncoderDirection EncoderDirectionY,
            /// Extra
            // Telemetry
            Telemetry telemetryUpdater
    ) {
        /// Initialization
        // Declare Drive Train Motor Location
        frontLeftDrive = hardwareMap.get(DcMotor.class, FL_NAME);
        backLeftDrive = hardwareMap.get(DcMotor.class, BL_NAME);
        frontRightDrive = hardwareMap.get(DcMotor.class, FR_NAME);
        backRightDrive = hardwareMap.get(DcMotor.class, BR_NAME);

        // Declare Launcher Locations
        flywheelMotor1 = hardwareMap.get(DcMotorEx.class, FW1_NAME);
        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, FW2_NAME);
        intakeMotor1 = hardwareMap.get(DcMotor.class, IN1_NAME);
        intakeMotor2 = hardwareMap.get(DcMotor.class, IN2_NAME);

        // Declare goBilda Pinpoint Odometry Computer
        //pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);

        // Telemetry
        telemetry = telemetryUpdater;

        // Set Drive Train Motor Directions
        frontLeftDrive.setDirection(FLDir);
        backLeftDrive.setDirection(BLDir);
        frontRightDrive.setDirection(FRDir);
        backRightDrive.setDirection(BRDir);

        // Set Launcher Motor Directions
        flywheelMotor1.setDirection(FW1Dir);
        flywheelMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor2.setDirection(FW2Dir);
        flywheelMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeMotor1.setDirection(IN1Dir);
        intakeMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor2.setDirection(IN2Dir);
        intakeMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize Pinpoint Odometry Computer
        //pinpoint.initialize();
        //pinpoint.setOffsets(OffsetX, OffsetY, distanceUnit);
        //pinpoint.setEncoderResolution(OdometryPodType);
        //pinpoint.setEncoderDirections(EncoderDirectionX, EncoderDirectionY);

        //pinpoint.resetPosAndIMU();
        Pose2D startPos = new Pose2D(distanceUnit, 0, 0, angleUnit, 0);
        //pinpoint.setPosition(startPos);
    }

    /// Functions
    public void MoveTo(double wishXPos, double wishYPos)
    {

    }

    public void RotateToHeading(double wishHeading)
    {
        telemetry.addData("Status", "Trying to turning");
        canMove = true;
        if ((heading > wishHeading + 3) || (heading < wishHeading - 3))
        {
            canMove = false;
            telemetry.addData("Status", "Turning");
            if (heading > wishHeading)
            {
                frontLeftDrive.setPower(autoSpeedMod);
                backLeftDrive.setPower(autoSpeedMod);
                frontRightDrive.setPower(-autoSpeedMod);
                backRightDrive.setPower(-autoSpeedMod);
            }
            else if  (heading < wishHeading)
            {
                frontLeftDrive.setPower(-autoSpeedMod);
                backLeftDrive.setPower(-autoSpeedMod);
                frontRightDrive.setPower(autoSpeedMod);
                backRightDrive.setPower(autoSpeedMod);
            }
        }
        else
        {
            telemetry.addData("Status", "Unacceptable Turn Conditions");
        }
    }
}