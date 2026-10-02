package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.config.DriveConfig;
import org.firstinspires.ftc.teamcode.drivetrain.Drive;

// base class for every opmode, loop and stop are final
// so the odometry update and motor stop always run in order
public abstract class RobotOpMode extends OpMode {

    protected Robot robot;
    private DriveSequence sequence;

    // override to change the config or the chassis type, call super.init first
    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, DriveConfig.ROBOT, Robot.Chassis.MECANUM);
        sequence = new DriveSequence();
        onInit();
    }

    // runs every loop while initializing, robot still held
    @Override
    public void init_loop() {
        robot.startUp();
        onInitLoop();
    }

    // refreshes odometry and steers at the target, then runs the opmode
    @Override
    public final void loop() {
        robot.drive().update();
        onLoop();
    }

    // stops the motors, always runs even if the opmode threw
    @Override
    public final void stop() {
        robot.shutDown();
        onStop();
    }

    /// hooks
    // one time setup, do not touch the motors, they are not held yet
    protected abstract void onInit();

    // runs until the driver presses play, this is where calibration waits
    protected void onInitLoop() {
    }

    // the body of the opmode, once per loop
    protected abstract void onLoop();

    // cleanup, the base class already stopped the motors
    protected void onStop() {
    }

    /// helpers
    // shorthand for robot.drive
    protected Drive drive() {
        return robot.drive();
    }

    // start pose, call once from onInit, applied automatically at the right
    // moment
    protected void setStartPose(double xInches, double yInches, double headingDegrees) {
        robot.setStartPose(xInches, yInches, headingDegrees);
    }

    // the autonomous step list, build it in onInit then run it every loop in
    // onLoop
    protected DriveSequence sequence() {
        sequence.attach(robot.drive());
        return sequence;
    }

    // adds this robot's telemetry, call from onLoop
    protected void publishTelemetry() {
        robot.addTelemetry(telemetry);
    }

    // wait until the robot arrives, returns immediately so keep calling it
    // every loop
    protected boolean isBusy() {
        return robot.drive().isBusy();
    }
}
