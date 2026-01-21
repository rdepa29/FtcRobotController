package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Robot: Basic Linear Autonomous OpMode Blue Side Short", group = "Linear OpMode")
public class BlueShortAutonomousOpMode_Linear extends LinearOpMode {
    @Override
    public void runOpMode()
    {
        OpModeHelper helper = new OpModeHelper(
                "FL", "FR", "BL", "BR",
                "FW1", "FW2", "IN1", "IN2",
                "PINPOINT",
                hardwareMap,
                DistanceUnit.MM, AngleUnit.DEGREES,
                -95.25, 19.05,
                DcMotor.Direction.REVERSE, DcMotor.Direction.REVERSE, DcMotor.Direction.FORWARD, DcMotor.Direction.REVERSE,
                DcMotor.Direction.FORWARD, DcMotor.Direction.FORWARD, DcMotor.Direction.FORWARD, DcMotor.Direction.FORWARD,
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED,
                telemetry
        );

        // Start Runtime Debugger
        ElapsedTime runtime = helper.runtime;

        // Declare Drive Train Motors
        DcMotor frontLeftDrive = helper.frontLeftDrive;
        DcMotor backLeftDrive = helper.backLeftDrive;
        DcMotor frontRightDrive = helper.frontRightDrive;
        DcMotor backRightDrive = helper.backRightDrive;

        // Flywheels and Launcher Feeder
        DcMotorEx flywheelMotor = (DcMotorEx) helper.flywheelMotor1;

        // goBilda Pinpoint Odometry Computer
        //GoBildaPinpointDriver pinpoint = helper.pinpoint;
        double xPos = helper.xPos;
        double yPos = helper.yPos;
        DistanceUnit du = DistanceUnit.MM;
        AngleUnit au = AngleUnit.DEGREES;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        telemetry.addData("Status", "Driving Backwards");
        telemetry.update();
        runtime.reset();

        // Drive Backwards
        while (opModeIsActive() && runtime.seconds() < 4)
        {
            frontLeftDrive.setPower(-helper.autoSpeedMod);
            backLeftDrive.setPower(-helper.autoSpeedMod);
            frontRightDrive.setPower(-helper.autoSpeedMod);
            backRightDrive.setPower(-helper.autoSpeedMod);
        }

        frontLeftDrive.setPower(0.0);
        backLeftDrive.setPower(0.0);
        frontRightDrive.setPower(0.0);
        backRightDrive.setPower(0.0);

        // Start Launcher Flywheel
        flywheelMotor.setVelocity(660);

        sleep(6000);

        telemetry.addData("Status", "Starting Feeders");
        telemetry.update();
        runtime.reset();

        // Start Feeders
        while (opModeIsActive() && runtime.seconds() < 8.5)
        {

        }

        // Turn off Flywheel and Feeders
        telemetry.addData("Status", "Stopping Feeders");
        telemetry.update();
        runtime.reset();

        telemetry.addData("Status", "Stopping Flywheel");
        telemetry.update();
        runtime.reset();

        flywheelMotor.setVelocity(0.0);

        telemetry.addData("Status", "Driving Left");
        telemetry.update();
        runtime.reset();

        // Drive Left
        while (opModeIsActive() && runtime.seconds() < 1.0)
        {
            frontLeftDrive.setPower(-helper.autoSpeedMod * 2);
            backLeftDrive.setPower(helper.autoSpeedMod * 2);
            frontRightDrive.setPower(-helper.autoSpeedMod * 2);
            backRightDrive.setPower(helper.autoSpeedMod * 2);
        }

        frontLeftDrive.setPower(0.0);
        backLeftDrive.setPower(0.0);
        frontRightDrive.setPower(0.0);
        backRightDrive.setPower(0.0);

        telemetry.addData("Status", "Stopping Autonomous Mode");
        telemetry.update();
    }
}
