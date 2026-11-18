package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

// two-stick tank drive: the left stick drives the "L" side, the right stick drives "R"
// standalone on purpose, RobotOpMode always builds a four wheel mecanum robot
// https://ftc-docs.firstinspires.org/en/latest/ftc_sdk/overview/index.html
@TeleOp(name = "Drive: Tank TeleOp", group = "01 Drive")
public class TankTeleOp extends OpMode {
    // names from the driver station configuration, case sensitive
    // add a second entry ("L2", "R2") when a side carries two motors
    private static final String[] LEFT_MOTORS = {"L"};
    private static final String[] RIGHT_MOTORS = {"R"};
    private static final String ARM_MOTOR = "Arm";
    private static final String ARM_SERVO = "ArmServo";

    // mirrored motors on one axle usually need one side reversed
    // flip here before rewiring a motor
    private static final DcMotor.Direction LEFT_DIRECTION = DcMotor.Direction.REVERSE;
    private static final DcMotor.Direction RIGHT_DIRECTION = DcMotor.Direction.FORWARD;
    private static final DcMotor.Direction ARM_DIRECTION = DcMotor.Direction.FORWARD;

    // arm and servo tuning constants
    private static final double ARM_POWER = 0.3;
    private static final double SERVO_START = 0.5;
    private static final double SERVO_STEP = 0.01; // change per loop tick (~20ms) while held

    // sticks rest near zero, anything under this is noise
    private static final double DEAD_ZONE = 0.05;

    // null entries are devices the configuration does not have
    private DcMotor[] left = new DcMotor[0];
    private DcMotor[] right = new DcMotor[0];
    private DcMotor arm;
    private Servo armServo;

    private double leftPower;
    private double rightPower;
    private double armPower;
    private double servoPosition = SERVO_START;
    private String missing = "";

    @Override
    public void init() {
        left = claim(LEFT_MOTORS, LEFT_DIRECTION);
        right = claim(RIGHT_MOTORS, RIGHT_DIRECTION);
        arm = claimMotor(ARM_MOTOR, ARM_DIRECTION);
        armServo = claimServo(ARM_SERVO, SERVO_START);

        missing = join(unresolved(LEFT_MOTORS, left), unresolved(RIGHT_MOTORS, right));
        if (arm == null) {
            missing = join(missing, ARM_MOTOR);
        }
        if (armServo == null) {
            missing = join(missing, ARM_SERVO);
        }

        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Left stick L, right stick R | D-pad U/D: Arm | D-pad L/R: ArmServo");
        reportMissing();
    }

    @Override
    public void loop() {
        // a gamepad reports -1 when the stick is pushed up, so it is flipped once here
        leftPower = stick(-gamepad1.left_stick_y);
        rightPower = stick(-gamepad1.right_stick_y);

        drive(left, leftPower);
        drive(right, rightPower);

        // D-pad up/down controls the Arm motor
        if (gamepad1.dpad_up) {
            armPower = ARM_POWER;
        } else if (gamepad1.dpad_down) {
            armPower = -ARM_POWER;
        } else {
            armPower = 0.0;
        }
        if (arm != null) {
            arm.setPower(armPower);
        }

        // D-pad left/right incrementally moves the ArmServo (0.0 to 1.0)
        if (gamepad1.dpad_right) {
            servoPosition += SERVO_STEP;
        } else if (gamepad1.dpad_left) {
            servoPosition -= SERVO_STEP;
        }
        servoPosition = Range.clip(servoPosition, 0.0, 1.0);
        if (armServo != null) {
            armServo.setPosition(servoPosition);
        }

        telemetry.addData("Sticks", "L %+.2f   R %+.2f", leftPower, rightPower);
        telemetry.addData("Arm", "Power %+.2f   Servo %.2f", armPower, servoPosition);
        reportMissing();
    }

    // looks up one side, applies direction and braking
    // a name the configuration does not have leaves a null instead of throwing
    private DcMotor[] claim(String[] names, DcMotor.Direction direction) {
        DcMotor[] motors = new DcMotor[names.length];
        for (int i = 0; i < names.length; i++) {
            motors[i] = claimMotor(names[i], direction);
        }
        return motors;
    }

    private DcMotor claimMotor(String name, DcMotor.Direction direction) {
        DcMotor motor;
        try {
            motor = hardwareMap.get(DcMotor.class, name);
        } catch (IllegalArgumentException notConfigured) {
            return null;
        }
        motor.setDirection(direction);
        // brake holds the robot still when both sticks are released
        // OpMode.stop() may not touch actuators since SDK 8.2, so this is what stops the wheels
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }

    private Servo claimServo(String name, double initialPosition) {
        Servo servo;
        try {
            servo = hardwareMap.get(Servo.class, name);
        } catch (IllegalArgumentException notConfigured) {
            return null;
        }
        servo.setPosition(initialPosition);
        return servo;
    }

    private void drive(DcMotor[] motors, double power) {
        for (DcMotor motor : motors) {
            if (motor != null) {
                motor.setPower(power);
            }
        }
    }

    // dead zone with a rescale, so the output still reaches a full 1
    private static double stick(double raw) {
        double magnitude = Math.abs(raw);
        if (magnitude <= DEAD_ZONE) {
            return 0.0;
        }
        return Math.signum(raw) * (magnitude - DEAD_ZONE) / (1.0 - DEAD_ZONE);
    }

    private static String unresolved(String[] names, DcMotor[] motors) {
        StringBuilder text = new StringBuilder();
        for (int i = 0; i < names.length; i++) {
            if (motors[i] == null) {
                if (text.length() > 0) {
                    text.append(", ");
                }
                text.append(names[i]);
            }
        }
        return text.toString();
    }

    private static String join(String first, String second) {
        if (first.isEmpty()) {
            return second;
        }
        return second.isEmpty() ? first : first + ", " + second;
    }

    private void reportMissing() {
        if (!missing.isEmpty()) {
            telemetry.addData("Missing", missing + " not in the configuration");
        }
    }
}
