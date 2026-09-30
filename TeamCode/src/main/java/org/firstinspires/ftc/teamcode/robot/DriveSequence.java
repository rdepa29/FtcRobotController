package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.drivetrain.Drive;
import org.firstinspires.ftc.teamcode.drivetrain.Pose;

// the whole autonomous model, a list of steps that advances one per loop so a
// step that runs long never loses your place
//
// use addWait for anything that is not a target, like a settle before a shooter
//
// https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
public class DriveSequence {

    // one step, only ever called from the robot's thread
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

    // hands it the drivetrain, must be called before run, sequence does this
    // for you
    public void attach(Drive drive) {
        this.drive = drive;
    }

    // appends a drive to this point facing this way
    public DriveSequence add(String name, double x, double y, double headingDegrees) {
        return add(new Target(new Pose(x, y, headingDegrees)), name);
    }

    // appends a drive to a pose
    public DriveSequence add(String name, Pose pose) {
        return add(new Target(pose), name);
    }

    // appends a relative turn, positive is left, from whatever heading the robot
    // has when the step actually begins
    public DriveSequence addTurn(String name, double degrees) {
        return add(new Turn(degrees), name);
    }

    // appends a fixed wait with the wheels stopped, for settling a feedback loop
    // cannot express
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

    // advances the sequence by at most one step, once per loop
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

    // true once every step is finished, safe to poll forever
    public boolean isDone() {
        return index < 0;
    }

    // name of the step currently running, or the last one run
    public String getCurrentStep() {
        if (index < 0 || index >= count) {
            return count == 0 ? "empty" : "done";
        }
        return names[index];
    }

    // 0 to 1, useful for a progress bar on the DS
    public double getProgress() {
        if (count == 0) {
            return 1.0;
        }
        return Math.max(0.0, Math.min(1.0, (double) index / count));
    }

    // total number of steps
    public int size() {
        return count;
    }

    // resets the step list only, the robot stays where it is
    public void reset() {
        index = -1;
        activeWait = null;
    }
}
