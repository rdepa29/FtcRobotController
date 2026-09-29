package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.drivetrain.Drive;
import org.firstinspires.ftc.teamcode.drivetrain.Pose;

/**
 * A list of "go here" steps that runs itself one step per loop.
 *
 * <p>This is the entire autonomous programming model. A routine is a sequence of
 * targets; the sequence notices when each one is finished and starts the next. There
 * is no step counter in your own code, no flag to reset, and no way to lose your place
 * if a step takes a different amount of time than expected, which it always does.
 *
 * <pre>
 *   sequence()
 *       .add("score low", 24, 24, 0)
 *       .add("grab",       60, 36, 90)
 *       .add("back away",  24, 24, 0);
 *
 *   // in onLoop():
 *   if (sequence().isDone()) { /* do the endgame thing *&#47; }
 *   else { sequence().run(); }
 * </pre>
 *
 * <p>Timing that cannot be expressed as a target, like a 300 ms settle before a
 * shooter, goes in with {@link #addWait}. The clock starts when the wait begins, not
 * when the OpMode starts, so a slow approach does not eat into it.
 */
public class DriveSequence {

    /** A single step. Implementations only ever get called from the robot's thread. */
    public interface Step {
        void begin(Drive drive);
    }

    private static final class Target implements Step {
        final Pose pose;

        Target(Pose pose) {
            this.pose = pose;
        }

        @Override
        public void begin(Drive drive) {
            drive.goTo(pose);
        }
    }

    private static final class Turn implements Step {
        final double degrees;

        Turn(double degrees) {
            this.degrees = degrees;
        }

        @Override
        public void begin(Drive drive) {
            drive.turnBy(degrees);
        }
    }

    private static final class Wait implements Step {
        final double seconds;
        private final ElapsedTime timer = new ElapsedTime();

        Wait(double seconds) {
            this.seconds = seconds;
        }

        @Override
        public void begin(Drive drive) {
            drive.stop();
            timer.reset();
        }

        boolean finished() {
            return timer.seconds() >= seconds;
        }
    }

    private final String[] names = new String[16];
    private final Step[] steps = new Step[16];
    private int count;

    private int index = -1;
    private Wait activeWait;
    private Drive drive;

    /**
     * Hands the sequence the drivetrain it will drive. Must be called before
     * {@link #run()}. {@link RobotOpMode#sequence()} does this for you.
     */
    public void attach(Drive drive) {
        this.drive = drive;
    }

    /**
     * Appends a "drive to this point and face this way" step.
     *
     * @param name shows up in telemetry while this step runs
     */
    public DriveSequence add(String name, double x, double y, double headingDegrees) {
        return add(new Target(new Pose(x, y, headingDegrees)), name);
    }

    /** Appends a step that drives to a {@link Pose}. */
    public DriveSequence add(String name, Pose pose) {
        return add(new Target(pose), name);
    }

    /**
     * Appends a relative turn. Positive degrees turn left, negative turn right, from
     * whatever heading the robot has when the step actually begins.
     */
    public DriveSequence addTurn(String name, double degrees) {
        return add(new Turn(degrees), name);
    }

    /**
     * Appends a step that does nothing for a fixed number of seconds. Wheels stopped.
     * Use for settling delays that a feedback loop cannot express.
     */
    public DriveSequence addWait(String name, double seconds) {
        return add(new Wait(seconds), name);
    }

    private DriveSequence add(Step step, String name) {
        if (count == names.length) {
            throw new IllegalStateException(
                    "DriveSequence holds " + names.length + " steps. Split the routine.");
        }
        names[count] = name;
        steps[count] = step;
        count++;
        return this;
    }

    /**
     * Advances the sequence by at most one step. Call once per loop.
     *
     * <p>Does nothing once the sequence is done, so a routine that has already finished
     * cannot drive the robot anywhere. That is the safety property that makes it safe
     * to leave this call in a loop that also does other things.
     */
    public void run() {
        if (isDone() || drive == null) {
            return;
        }
        if (index < 0) {
            index = 0;
            begin(0);
            return;
        }
        if (activeWait != null) {
            if (!activeWait.finished()) {
                return;
            }
            activeWait = null;
        } else if (drive.isBusy()) {
            return;
        }
        index++;
        if (index >= count) {
            index = -1;
            drive.stop();
            return;
        }
        begin(index);
    }

    private void begin(int i) {
        activeWait = steps[i] instanceof Wait ? (Wait) steps[i] : null;
        steps[i].begin(drive);
    }

    /** True when every step has finished. Safe to poll forever. */
    public boolean isDone() {
        return index < 0;
    }

    /** Name of the step currently running, or the last one run. */
    public String getCurrentStep() {
        if (index < 0 || index >= count) {
            return count == 0 ? "empty" : "done";
        }
        return names[index];
    }

    /** Fraction complete, 0 to 1. Useful for a progress bar on the DS. */
    public double getProgress() {
        if (count == 0) {
            return 1.0;
        }
        return Math.max(0.0, Math.min(1.0, (double) index / count));
    }

    /** Total number of steps. */
    public int size() {
        return count;
    }

    /**
     * Starts the sequence over from the first step. The robot does not move back to
     * the start of the routine; only the step list resets.
     */
    public void reset() {
        index = -1;
        activeWait = null;
    }
}
