package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.DriveConfig;
import org.firstinspires.ftc.teamcode.drivetrain.DifferentialDrive;
import org.firstinspires.ftc.teamcode.drivetrain.Drive;
import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.drivetrain.EncoderOdometry;
import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.drivetrain.PinpointOdometry;
import org.firstinspires.ftc.teamcode.drivetrain.Pose;

// builds each module the first time an opmode asks for it, so one that only
// wants the drivetrain never spins up the intake
//
// one apk per team means the graph gets built at runtime anyway
//
// https://ftc-docs.firstinspires.org/en/latest/ftc_sdk/overview/index.html
public class Robot {

    // which mixing to build, change this one line when the drivetrain changes
    public enum Chassis {
        MECANUM,
        DIFFERENTIAL
    }

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final DriveConfig config;
    private final Chassis chassis;

    private Drivetrain drivetrain;
    private Odometry odometry;
    private Drive drive;
    private Pose startPose;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, DriveConfig config, Chassis chassis) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.config = config;
        this.chassis = chassis;
    }

    // lazy modules
    public Drivetrain drivetrain() {
        if (drivetrain == null) {
            drivetrain = chassis == Chassis.DIFFERENTIAL
                    ? new DifferentialDrive(hardwareMap, config)
                    : new MecanumDrive(hardwareMap, config);
            if (!drivetrain.isAvailable()) {
                telemetry.addData("Robot",
                        "WARNING: drivetrain is missing motors. Check the names in DriveConfig "
                                + "against hardware.xml.");
            }
        }
        return drivetrain;
    }

    // where the robot thinks it is, built on first use
                PinpointOdometry pinpoint = new PinpointOdometry(hardwareMap, config);
                if (pinpoint.isAvailable()) {
                    odometry = pinpoint;
                }
            }
            if (odometry == null) {
                odometry = new EncoderOdometry(hardwareMap, drivetrain(), config);
            }
        }
        return odometry;
    }

    // the high level drivetrain, built on first use, pulls in both of the above
    public Drive drive() {
        if (drive == null) {
            drive = new Drive(drivetrain(), odometry(), config);
        }
        return drive;
    }

    // lifecycle
                // recalibrate first, the other order throws the pose away
                pinpoint.recalibrate();
                if (startPose != null) {
                    pinpoint.setPose(startPose);
                }
            }
            return;
        }
        // the encoder fallback has nothing to calibrate, so apply it once and be done
        if (startPose != null) {
            current.setPose(startPose);
            startPose = null;
        }
    }

    // called at the end of every opmode, stops the motors unconditionally
    public void shutDown() {
        if (drive != null) {
            drive.stop();
        } else if (drivetrain != null) {
            drivetrain.stop();
        }
    }

    public DriveConfig getConfig() {
        return config;
    }

    public Chassis getChassis() {
        return chassis;
    }

    public void addTelemetry(Telemetry telemetryTo) {
        telemetryTo.addData("Chassis", chassis);
        if (drive != null) {
            drive.addTelemetry(telemetryTo);
        } else {
            drivetrain().addTelemetry(telemetryTo);
            odometry().addTelemetry(telemetryTo);
        }
    }
}
