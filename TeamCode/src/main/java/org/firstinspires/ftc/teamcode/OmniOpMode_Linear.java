package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Omni Linear OpMode", group = "Linear OpMode")
public class OmniOpMode_Linear extends LinearOpMode {
    @Override
    public void runOpMode() {
        OpModeHelper helper = new OpModeHelper(
                "FL", "FR", "BL", "BR",
                "FW1", "FW2", "IN1", "IN2",
                "PINPOINT",
                hardwareMap,
                DistanceUnit.MM, AngleUnit.DEGREES,
                -95.25, 19.05,
                DcMotor.Direction.REVERSE, DcMotor.Direction.REVERSE, DcMotor.Direction.REVERSE, DcMotor.Direction.REVERSE,
                DcMotor.Direction.FORWARD, DcMotor.Direction.FORWARD, DcMotor.Direction.REVERSE, DcMotor.Direction.REVERSE,
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
        DcMotorEx flywheelMotor1 = (DcMotorEx) helper.flywheelMotor1;
        DcMotorEx flywheelMotor2 = (DcMotorEx) helper.flywheelMotor2;

        DcMotorEx intakeMotor1 = (DcMotorEx) helper.intakeMotor1;
        DcMotorEx intakeMotor2 = (DcMotorEx) helper.intakeMotor2;

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

        while (opModeIsActive()) {
            // weird controller shit
            double max;

            double move = gamepad2.right_stick_x; // Switch these (idk why but it only likes it like ts)
            double strafe = gamepad2.left_stick_x;
            double turn = -gamepad2.left_stick_y; // Switch these (idk why but it only likes it like ts)

            double frontLeftPower  = move - strafe + turn;
            double frontRightPower = move - strafe - turn;
            double backLeftPower   = move + strafe + turn;
            double backRightPower  = move + strafe - turn;

            // Calculate xPos and yPos with Odometry Computer
            //pinpoint.update();
            //xPos = pinpoint.getPosX(du);
            //yPos = pinpoint.getPosY(du);
            //helper.heading = pinpoint.getHeading(au);

            // weird math shit
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            // Get Buttons Down
            // Shooter
            double gamepadSquare1 = gamepad1.x ? 1.0 : 0.0;
            double gamepadCircle1 = gamepad1.b ? 1.0 : 0.0;
            double gamepadCross1 = gamepad1.a ? 1.0 : 0.0;
            double gamepadTriangle1 = gamepad1.y ? 1.0 : 0.0;
            double dpadU1 = gamepad1.dpad_up ? 1.0 : 0.0;
            double dpadR1 = gamepad1.dpad_right ? 1.0 : 0.0;
            double dpadD1 = gamepad1.dpad_down ? 1.0 : 0.0;
            double dpadL1 = gamepad1.dpad_left ? 1.0 : 0.0;
            double bumperR1 = gamepad1.right_bumper ? 1.0 : 0.0;
            double bumperL1 = gamepad1.left_bumper ? 1.0 : 0.0;
            // Driver
            double gamepadSquare2 = gamepad2.x ? 1.0 : 0.0;
            double gamepadCircle2 = gamepad2.b ? 1.0 : 0.0;
            double gamepadCross2 = gamepad2.a ? 1.0 : 0.0;
            double gamepadTriangle2 = gamepad2.y ? 1.0 : 0.0;
            double dpadU2 = gamepad2.dpad_up ? 1.0 : 0.0;
            double dpadR2 = gamepad2.dpad_right ? 1.0 : 0.0;
            double dpadD2 = gamepad2.dpad_down ? 1.0 : 0.0;
            double dpadL2 = gamepad2.dpad_left ? 1.0 : 0.0;
            double bumperR2 = gamepad2.right_bumper ? 1.0 : 0.0;
            double bumperL2 = gamepad2.left_bumper ? 1.0 : 0.0;

            // Set Drive Train Motor Power
            if (helper.canMove)
            {
                frontLeftDrive.setPower(frontLeftPower * helper.speedMod);
                frontRightDrive.setPower(frontRightPower * helper.speedMod);
                backLeftDrive.setPower(backLeftPower * helper.speedMod);
                backRightDrive.setPower(backRightPower * helper.speedMod);
            }

            // Shooter Buttons
            if (gamepadCross1 == 1.0) // Motor OFF
            {
                flywheelMotor1.setVelocity(0);
                flywheelMotor2.setVelocity(0);
            }
            else if (dpadU1 == 1.0) // Motor ON
            {
                flywheelMotor1.setVelocity(680);
                flywheelMotor2.setVelocity(680);
            }
            else if (dpadD1 == 1.0) // Motor HIGH
            {
                flywheelMotor1.setVelocity(800);
                flywheelMotor2.setVelocity(800);
            }

            // Drivetrain Buttons
            if (bumperR2 == 1.0)
            {
                helper.speedMod = 0.2;
            }
            else if (bumperR2 == 0.0)
            {
                helper.speedMod = 1.0;
            }

            if (dpadR2 == 1.0)
            {
                intakeMotor1.setPower(1.0);
                intakeMotor2.setPower(1.0);
            }
            else if (dpadL2 == 1.0)
            {
                intakeMotor1.setPower(-1.0);
                intakeMotor2.setPower(-1.0);
            }
            else if (dpadD2 == 1.0)
            {
                intakeMotor1.setPower(0.0);
                intakeMotor2.setPower(0.0);
            }


            telemetry.addData("Runtime", runtime.toString());
            telemetry.addData("Move", move);
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Turn", turn);
            telemetry.addData("Flywheel Motor 1 TPS", flywheelMotor1.getVelocity());
            telemetry.addData("Flywheel Motor 2 TPS", flywheelMotor2.getVelocity());
            telemetry.addData("Odometry Computer X Position", xPos);
            telemetry.addData("Odometry Computer Y Position", yPos);
            telemetry.addData("Odometry Computer Heading", helper.heading);
            telemetry.update();
        }
    }
}