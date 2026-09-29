# TeamCode

Drivetrain framework for the 2026-2027 **BIOBUZZ** season, FTC SDK `12.0`.

The goal of this module is narrow: make the robot *move where you tell it to move*, in as
few concepts as possible. Everything else in a robot is complicated. This part should not be.

```java
drive.setPower(forward, strafe, turn);   // drive like a video game
drive.goTo(x, y, heading);               // drive to a spot on the field
```

Both are the same class. There is no motion class to learn, no path to compile, and no
`Motion` to reconcile. If you can say "go to the center of C3, facing the loading zone",
you have already written the call.

---

## Contents

1. [Writing your first OpMode](#writing-your-first-opmode)
2. [Coordinates](#coordinates)
3. [API reference](#api-reference)
4. [Configuring the robot](#configuring-the-robot)
5. [Before you can actually run this](#before-you-can-actually-run-this)
6. [What this does not do yet](#what-this-does-not-do-yet)
7. [How it is put together](#how-it-is-put-together)
8. [References](#references)

---

## Writing your first OpMode

Every OpMode extends `RobotOpMode`. You implement **two** methods and you never touch the
hardware map, the motors, or the odometry update loop.

```java
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.config.FieldConfig;
import org.firstinspires.ftc.teamcode.config.FieldConfig.Tile;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;

@TeleOp(name = "Drive: My OpMode", group = "01 Drive")
public class MyOpMode extends RobotOpMode {

    @Override
    protected void onInit() {
        // Runs once, when INIT is pressed. Build your routines here.
        sequence()
                .add("to A1", FieldConfig.cell(Tile.A1, 0))
                .addTurn("turn around", 180)
                .addWait("settle", 0.5);
    }

    @Override
    protected void onLoop() {
        // Runs over and over until STOP. Do the work here.
        sequence().run();
        publishTelemetry();
    }
}
```

Two rules make this reliable:

- **Put your setup in `onInit`, not in `onLoop`.** `onInit` runs once. Code at the top of
  `onLoop` runs ~50 times a second and is a common source of "works once, then explodes"
  bugs.
- **Call `publishTelemetry()` in `onLoop`.** It reports the pose and any active target.

### The four hooks

| Method | When it runs | Use it for |
| --- | --- | --- |
| `onInit()` | once, at INIT | Declaring sequences and constants |
| `onInitLoop()` | repeatedly, until PLAY | Waiting for device calibration |
| `onLoop()` | repeatedly, until STOP | The actual routine |
| `onStop()` | once, at STOP | Cleanup, if you need it |

`onInitLoop` exists because the 30-second AUTO period starts the instant PLAY is pressed.
A goBILDA Pinpoint IMU needs about a quarter second of stillness to calibrate, and that
calibration cannot happen inside AUTO. The base class already calls `robot.startUp()` from
`init_loop`, so the Pinpoint is calibrated for you before AUTO begins.

### Available to you in any OpMode

| Name | Type | What it is |
| --- | --- | --- |
| `drive()` | `Drive` | The drivetrain. All movement goes through it. |
| `sequence()` | `DriveSequence` | A list of waypoints that advances on its own. |
| `setStartPose(x, y, h)` | `void` | Tells the robot where it is on the field. |
| `publishTelemetry()` | `void` | Pushes pose and target to the Driver Station. |
| `isBusy()` | `boolean` | Is a `goTo` still in progress? |
| `telemetry` | `Telemetry` | The SDK telemetry object, if you want raw lines. |
| `robot` | `Robot` | The module holder. Rarely needed directly. |

### Teleop driving

Use direct power for anything a driver is doing. It ignores odometry, so it never fights
the position controller and it never drifts.

```java
double forward = -gamepad1.left_stick_y;
double strafe  =  gamepad1.left_stick_x;
double turn    =  gamepad1.right_stick_x;

if (gamepad1.a) {
    drive().driveField(forward, strafe, turn);   // heading-locked to the field
} else {
    drive().setPower(forward, strafe, turn);     // robot-relative
}
```

`setPower` is robot-relative and `driveField` is field-relative. The difference shows up
the moment the robot turns: with `setPower`, pushing the stick forward continues in the new
heading. With `driveField`, pushing forward always goes toward field `+Y`.

See `opmodes/DriveTeleOp.java` for a full version with the field-relative toggle and
dead-zone handling.

---

## Coordinates

Every position in this codebase is **inches on a 144 x 144 inch field**.

| Fact | Value |
| --- | --- |
| Field size | 144 x 144 inches |
| Tile size | 24 inches, 6 x 6 grid |
| Origin | lower-left corner, `(0, 0)` |
| `(144, 144)` | upper-right corner |
| Heading `0` | facing along field `+Y` |
| Heading `+90` | facing along field `+X` |
| Positive rotation | counter-clockwise |

The tile grid uses the letters the competition manual uses, `A`-`F` across and `1`-`6` up.
`FieldConfig.cell()` turns a tile into a position so you never have to compute one:

```java
import org.firstinspires.ftc.teamcode.config.FieldConfig;
import org.firstinspires.ftc.teamcode.config.FieldConfig.Tile;

drive().goTo(FieldConfig.cell(Tile.C3));            // center of C3, keep current heading
drive().goTo(FieldConfig.cell(Tile.C3, 90));        // center of C3, facing +X
drive().goTo(FieldConfig.tileX(Tile.C3), FieldConfig.tileY(Tile.C3));
```

> The origin corner and the direction of increasing row numbers are **not yet confirmed**
> against a real field. They are consistent and the math is correct, but if your robot
> drives to the wrong place, this table is the first thing to check, not the control loop.
> See `FieldConfig` for the specific values still marked `MEASURE`.

### The rotation gotcha

FTC's rotation is **not** the usual math convention. Rotating a field vector into the
robot frame is:

```
[[ sin,  cos],
 [-cos,  sin]]
```

which is the *transpose* of the robot-to-field matrix. Swapping these compiles fine,
produces believable telemetry, and is the single most common cause of "field-relative
driving is rotated 90 degrees" bugs. Both directions are written out explicitly in
`Drive.java` so they are easy to compare.

---

## API reference

### `Drive` — movement

```java
void setPower(double forward, double strafe, double turn)
void driveField(double forward, double strafe, double turn)
void stop()

void goTo(double xInches, double yInches)
void goTo(double xInches, double yInches, double headingDegrees)
void goTo(Pose pose)
void turnTo(double headingDegrees)
void turnBy(double degrees)

void update()
boolean isBusy()
void cancel()

Pose getPose()
void setPose(double x, double y, double heading)
void setPose(Pose pose)
void resetEncodersAndPose()

double getPositionError()     // inches remaining
double getHeadingError()      // degrees remaining
Drivetrain getDrivetrain()
Odometry getOdometry()
void addTelemetry(Telemetry telemetry)
```

- `forward`, `strafe`, `turn` are all in `-1.0 .. 1.0`. **Positive strafe is the robot's
  left.** `turn` is counter-clockwise.
- `goTo`, `turnTo`, and `turnBy` are **non-blocking**. They set a target and return
  immediately. The base class calls `update()` for you every loop.
- Calling any `setPower` or `driveField` **cancels** an active `goTo`. That is deliberate:
  the driver has taken control, so the autonomous target should not fight them.
- `goTo` to a target already within `positionToleranceInches` (default 2.0) does nothing and
  reports not busy. It is a no-op rather than a jitter around the goal.
- There is no `goTo(x, y, heading, tolerance)` overload. Tolerance is tuned in one place,
  `DriveConfig`, rather than at every call site.

### `DriveSequence` — multi-step autonomous

```java
DriveSequence add(String name, double x, double y, double headingDegrees)
DriveSequence add(String name, Pose pose)
DriveSequence addTurn(String name, double degrees)
DriveSequence addWait(String name, double seconds)

void run()                  // call every loop; advances one step
boolean isDone()
String getCurrentStep()     // name of the active step, "" when done
double getProgress()        // 0.0 to 1.0
int size()
void reset()
```

Methods chain, so a whole routine is one expression:

```java
sequence()
    .add("to A1", FieldConfig.cell(Tile.A1, 0))
    .add("to garden", FieldConfig.cell(Tile.A3, 90))
    .addTurn("face hive", -90)
    .addWait("release", 0.4);
```

Every step has a name, which makes it obvious on the Driver Station which one is running
when something misbehaves. The sequence holds no state that a re-run will not reset, so
`reset()` is enough to run the same routine again.

### `Pose` — position

```java
double getX(); double getY(); double getHeading()
double distanceTo(Pose other)
double bearingTo(Pose other)
double headingErrorTo(Pose other)
Pose translate(double dx, double dy)
Pose projectForward(double forwardInches)
Pose projectRight(double rightInches)
Pose2D toPose2D()
static Pose fromPose2D(Pose2D pose2D)
```

Units are fixed at inches and degrees. This wraps the SDK's `Pose2D` and reports which
units it is holding, so a value never gets read as the wrong dimension by accident.

### `Drivetrain` — the mixing layer

```java
void drive(double pitch, double roll, double yaw)   // -1..1 per axis
void stop(); void resetEncoders()
DriveMotor[] getMotors(); String getName(); boolean isAvailable()
```

Two implementations: `MecanumDrive` and `DifferentialDrive`. This interface is deliberately
narrow so that changing chassis does not require touching the control loop above it.

### `Odometry` — where the robot thinks it is

```java
void update(); Pose getPose()
double getXVelocity(); double getYVelocity(); double getYawVelocity()
void setPose(Pose pose); String getName(); boolean isAvailable()
```

Two implementations: `PinpointOdometry` (goBILDA two-pod dead-reckoning) and
`EncoderOdometry` (wheel encoders plus the hub IMU, used as a fallback).

### `MathUtils` — pure math

```java
double clamp(v, min, max);  double clampToMax(v, max);  double sign(v)
double wrapDegrees(deg);    double headingError(current, target)
void normalize(double[] powers, double max)
boolean nearlyEqual(a, b, tolerance)
double applyDeadZone(value, deadZone);  double signedSquare(value)
```

No SDK types, so this is the one file in the module that can be unit tested without a robot.

---

## Configuring the robot

All hardware and tuning lives in `config/DriveConfig.java` as a single static instance,
`DriveConfig.ROBOT`. You should not be constructing your own.

```java
// Motor names must match the Driver Station configuration exactly, including case.
String frontLeft = "FL";
DcMotor.Direction frontLeftDirection = DcMotor.Direction.REVERSE;
```

| Setting | Default | Meaning |
| --- | --- | --- |
| `frontLeft` / `frontRight` / `backLeft` / `backRight` | `FL` `FR` `BL` `BR` | Names from the hardware config |
| `frontLeftDirection` etc. | `REVERSE FORWARD REVERSE FORWARD` | Per-motor direction |
| `zeroPowerBehavior` | `BRAKE` | What a zero-power command does |
| `mirrorStrafe` | `false` | Set `true` if the robot strafes the wrong way |
| `gearRatio` | `1.0` | **Placeholder. Must be measured.** |
| `wheelRadiusInches` | `2.0` | Drive wheel radius |
| `trackWidthInches` | `15.0` | Left-to-right wheel distance |
| `wheelBaseInches` | `13.0` | Front-to-back wheel distance |
| `motorTicksPerRevolution` | `28.0` | From the motor spec sheet |
| `deadZone` | `0.05` | Gamepad dead zone |
| `maxPower` | `1.0` | Ceiling applied to every command |
| `autoPowerScale` | `1.0` | Scales down motor power in autonomous |
| `translationalGain` | `0.09` | P gain on position error |
| `headingGain` | `0.045` | P gain on heading error |
| `headingGainDamp` | `0.004` | D gain on heading error |
| `positionToleranceInches` | `2.0` | "Arrived" threshold |
| `headingToleranceDegrees` | `2.0` | "Arrived" threshold |
| `usePinpoint` | `true` | Falls back to encoders if absent |
| `pinpoint` | `"pinpoint"` | Name from the hardware config |
| `podXOffsetInches` | `-3.75` | Sideways offset of the forward pod |
| `podYOffsetInches` | `0.75` | Forward offset of the strafe pod |

Field geometry and object dimensions live in `config/FieldConfig.java`, and are documented
inline against the sections of the competition manual they come from.

### The control law, and why there is no `I`

`goTo` is a proportional controller on position with a small derivative term on heading.
There is deliberately **no integral term.** An integral term converges beautifully on the
field you tuned it on and badly on every other one, which is a bad trade for a competition
robot that will be pushed around. If a routine overshoots, raise `translationalGain` or
lower `autoPowerScale` before reaching for an integral term.

---

## Before you can actually run this

This module compiles and its control loop is verified in simulation, but it has **never run
on a robot**, because the robot does not exist yet and two things are missing.

**1. There is no hardware configuration.** `.TeamCode/hardware.xml` does not exist, so
nothing knows what the motors are called or which hub they are on. Until it does, the
robot will start and `drivetrain().isAvailable()` will be `false`, meaning every motor
command is a safe no-op rather than a crash.

**2. Several values are placeholders, not measurements.** These are the ones that will make
a robot drive wrong rather than fail loudly:

- `gearRatio` is `1.0`. Encoder-based odometry is **completely wrong** until this is the
  real gear ratio. `EncoderOdometry` is only a fallback, so this mostly matters if the
  Pinpoint is not wired up.
- `wheelRadiusInches` and `trackWidthInches` are estimates.
- Motor directions are assumed, not confirmed. If the robot spins instead of driving
  forward, flip the directions before touching any gains.
- `podXOffsetInches` / `podYOffsetInches` are estimates.

### Bring-up order

Once there is a robot to test on, do these in order. Skipping ahead is how a robot ends up
driving confidently to the wrong tile.

1. Confirm the four motor names resolve and `drivetrain().isAvailable()` is `true`.
2. Set the robot on blocks. Run a teleop and check that forward is forward, and that
   strafing does not rotate the robot. Fix directions; set `mirrorStrafe` if needed.
3. Pinpoint only: rotate the robot in place. Position should stay within about 4 inches. If
   it swings, a pod offset has the wrong sign — see the goBILDA guide in the references.
4. Tap a wheel and watch the encoder count in the right direction and magnitude.
5. `goTo` a point 24 inches away, robot on the floor. Tune `translationalGain` and
   `autoPowerScale` before anything else.
6. Only then trust `FieldConfig.cell()` coordinates, and confirm the origin corner against
   the real field.

---

## What this does not do yet

Known gaps, so nobody spends an afternoon rediscovering them:

- **No vision localization.** This is the big one. The SDK ships all four BIOBUZZ AprilTag
  clusters, but FIRST published every cluster's field position as `{0,0,0}` with identity
  orientation. `AprilTagDetection.robotPose` will not return a usable absolute field
  position out of the box. Odometry is the only reliable position source right now, which
  is why Pinpoint is the default rather than an optional extra.
- **No path following.** `goTo` drives a straight line to a point. It does not curve around
  obstacles, hug walls, or follow a spline. Multi-step sequences of `goTo` targets are the
  intended substitute until that exists.
- **`DifferentialDrive` is untested.** It mixes correctly, but nothing has driven on it. It
  discards strafe, which means a `goTo` to a lateral target will not converge the way it
  does on Mecanum.
- **`Pidf` is unused.** It is a textbook PIDF controller sitting in `util/`. Nothing calls
  it. It is there for the lifters and flywheels that arrive with the hardware.
- **Field coordinates are unconfirmed.** See [Coordinates](#coordinates).
- **No `hardware.xml`, no `local.properties`.** A full `assembleDebug` has not been run
  against the Android SDK, so resource processing and dexing are unverified. The Java
  sources do compile cleanly against the SDK 12 jars.

---

## How it is put together

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
├── config/
│   ├── DriveConfig            hardware names, directions, gains, tolerances
│   └── FieldConfig            field geometry, Tile enum, object dimensions
├── drivetrain/
│   ├── Pose                   inches + degrees, wraps the SDK Pose2D
│   ├── DriveMotor             one motor, tolerant of missing hardware
│   ├── Drivetrain             interface
│   ├── MecanumDrive           four-wheel mecanum mixing
│   ├── DifferentialDrive      tank mixing, discards strafe
│   ├── Odometry               interface
│   ├── PinpointOdometry       goBILDA two-pod dead reckoning
│   ├── EncoderOdometry        encoders + hub IMU fallback
│   └── Drive                  the public API: setPower, goTo, turnTo
├── opmodes/
│   ├── DriveTeleOp            direct driving, field-relative toggle
│   └── ExampleAuto            four-step sequence, proves the loop closes
├── robot/
│   ├── Robot                  module assembly, lazy, chassis + odometry choice
│   ├── RobotOpMode            base class, owns the lifecycle and update loop
│   └── DriveSequence          list of waypoints
└── util/
    ├── MathUtils              pure math
    └── Pidf                   unused PIDF controller
```

`loop()` and `stop()` are `final` in `RobotOpMode`. A subclass cannot accidentally skip the
odometry update or leave the motors running, which is the kind of mistake that costs a
match rather than a practice.

`Robot` builds its modules lazily. Every team ships one APK, so all of this loads together;
asking for `drive()` should not drag in hardware a routine never touches.

### Safety properties worth knowing

- **A missing motor is survivable.** `DriveMotor` reports `isPresent() == false` and every
  command becomes a no-op, instead of throwing out of `hardwareMap.get` and ending the run.
- **Odometry update cannot be skipped.** It happens in the final `loop()`.
- **`setStartPose` is deferred.** It is applied once the odometry is actually ready, so
  calling it in `onInit` cannot be silently overwritten by a later Pinpoint reset.
- **Manual driving cancels autonomous.** `setPower` clears any `goTo` target.

---

## References

Every source file in this module carries its own `References` section in its class javadoc,
tailored to what that file depends on. All links below were checked reachable.

| Topic | Link |
| --- | --- |
| SDK Javadoc (`12.0.0`) | <https://javadoc.io/doc/org.firstinspires.ftc/RobotCore/12.0.0/index.html> |
| SDK overview, OpMode lifecycle | <https://ftc-docs.firstinspires.org/en/latest/ftc_sdk/overview/index.html> |
| BIOBUZZ Competition Manual | <https://ftc-resources.firstinspires.org/ftc/game/manual> |
| goBILDA Pinpoint User Guide | <https://www.gobilda.com/content/user_manuals/3110-0002-0001%20User%20Guide.pdf> |
| goBILDA Pinpoint driver source | <https://github.com/goBILDA-Official/FtcRobotController-Add-Pinpoint> |
| gm0 holonomic drivetrains | <https://gm0.org/en/latest/docs/common-mechanisms/drivetrains/holonomic.html> |
| gm0 mecanum drive tutorial | <https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html> |

Useful manual sections, all in the BIOBUZZ manual above:

| Section | Contents |
| --- | --- |
| 9.2 | Field dimensions and accuracy |
| 9.3 | Areas, zones, markings |
| 9.4 | Tile coordinates (the `A`-`F` / `1`-`6` grid) |
| 9.5 | Alliance area |
| 9.6 | Hive structure |
| 9.7 | Flower |
| 9.8 | Scoring elements |
| 9.9 | AprilTags |
