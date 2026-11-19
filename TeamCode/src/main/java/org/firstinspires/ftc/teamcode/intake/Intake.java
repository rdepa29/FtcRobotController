package org.firstinspires.ftc.teamcode.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

public class Intake {
    private final DcMotor motor;

    public Intake(HardwareMap hardwareMap, RobotConfig config) {
        motor = hardwareMap.get(DcMotor.class, config.intake);
        motor.setDirection(config.intakeDirection);
        motor.setZeroPowerBehavior(config.zeroPowerBehavior);
    }

    public void setPower(double power) { motor.setPower(power); }

    public void in() { motor.setPower(1.0); }

    public void out() { motor.setPower(-1.0); }

    public void stop() { motor.setPower(0.0); }
}
