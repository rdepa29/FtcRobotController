package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.config.DriveConfig;
import org.firstinspires.ftc.teamcode.drivetrain.Drive;

/**
 * Base class for every OpMode, so that no OpMode can forget to stop the motors.
 *
 * <p>An OpMode extends this and fills in the four hooks. The lifecycle methods are
 * {@code final}, which means the drivetrain update and the safety stop always run in
 * the right order no matter what the OpMode body does.
 *
 * <pre>
 * public class ExampleAuto extends RobotOpMode {
 *     &#64;Override protected void onInit() { }
 *     &#64;Override protected void onInitLoop() { }
 *     &#64;Override protected void onLoop() {
 *         if (!sequence.isDone()) { sequence.run(); }
 *     }
 * }
 * </pre>
 *
 * <h2>Why OpMode and not LinearOpMode</h2>
 *
 * <p>{@code LinearOpMode} is a nice idea that does not fit a competition robot. It
 * blocks inside {@code runOpMode()}, so a loop written as a list of steps cannot check
 * a button, cannot display telemetry that updates, and can only be aborted by throwing.
 * More importantly it gives no {@code init_loop()}, and a Pinpoint needs a few seconds
 * of stillness to calibrate that have to be spent somewhere other than inside the
 * 30 second autonomous period. Writing the hooks out is a few more lines and it is
 * worth it.
 *
 * <h2>References</h2>
 *
 *  OpMode lifecycle, per the SDK:
 *    https://javadoc.io/doc/org.firstinspires.ftc/RobotCore/12.0.0/index.html
 *
 *    init()       once, when INIT is pressed.
 *    init_loop()  repeatedly, until PLAY. The only safe place to wait on device
 *                 calibration, because it is not inside the 30 second AUTO.
 *    loop()       repeatedly, until STOP.
 *    stop()       once. An OpMode can still touch hardware here, but the robot is
 *                 already disabled by the time it runs, so do not rely on it.
 *
 *  loop() and stop() are final in this base class so a subclass cannot skip the
 *  odometry update or leave the motors running.
 */
public abstract class RobotOpMode extends OpMode {

    protected Robot robot;
    private DriveSequence sequence;

    /**
     * Builds the robot. Override to change the config or the chassis type; call
     * {@code super.init()} first.
     */
    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, DriveConfig.ROBOT, Robot.Chassis.MECANUM);
        sequence = new DriveSequence();
        onInit();
    }

    /** Runs every loop while the OpMode is initializing, with the robot still held. */
    @Override
    public void init_loop() {
        robot.startUp();
        onInitLoop();
    }

    /** Refreshes odometry and steers toward any active target, then runs the OpMode. */
    @Override
    public final void loop() {
        robot.drive().update();
        onLoop();
    }

    /** Stops the motors. Always runs, even if the OpMode threw. */
    @Override
    public final void stop() {
        robot.shutDown();
        onStop();
    }

    // ------------------------------------------------------------------
    // Hooks
    // ------------------------------------------------------------------

    /** One time setup. Do not touch the motors here; they are not held yet. */
    protected abstract void onInit();

    /** Runs until the driver presses play. The place to wait for calibration. */
    protected void onInitLoop() {
    }

    /** The body of the OpMode, once per loop. */
    protected abstract void onLoop();

    /** Cleanup. Do not stop the motors here; the base class already has. */
    protected void onStop() {
    }

    // ------------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------------

    /** Shorthand for {@code robot.drive()}. */
    protected Drive drive() {
        return robot.drive();
    }

    /**
     * Tells the robot where it is starting. Call once from {@link #onInit()}. The pose
     * is applied automatically, at the correct moment, by the base class.
     */
    protected void setStartPose(double xInches, double yInches, double headingDegrees) {
        robot.setStartPose(xInches, yInches, headingDegrees);
    }

    /**
     * The autonomous step list. Call it in {@link #onInit()} to build the routine, then
     * call {@link DriveSequence#run()} every loop in {@link #onLoop()} and check
     * {@link DriveSequence#isDone()}.
     */
    protected DriveSequence sequence() {
        sequence.attach(robot.drive());
        return sequence;
    }

    /** Adds this robot's telemetry to the OpMode's telemetry. Call from onLoop. */
    protected void publishTelemetry() {
        robot.addTelemetry(telemetry);
    }

    /**
     * Convenience for the common "wait until the robot arrives" shape. Returns
     * immediately; keep calling it every loop.
     */
    protected boolean isBusy() {
        return robot.drive().isBusy();
    }
}
