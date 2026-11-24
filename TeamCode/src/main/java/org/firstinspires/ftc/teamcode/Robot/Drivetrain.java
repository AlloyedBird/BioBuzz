package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Drivetrain for FTC mecanum robots using a SparkFun OTOS.
 *
 * THREADING MODEL
 *  - TeleOp: everything runs on the calling (main) thread. No control thread.
 *  - Autonomous: call moveToPose(...) repeatedly from your OpMode loop. The first call starts a
 *    single 50 Hz control thread, which is then the ONLY thing that touches the OTOS and the
 *    motors until stop() is called. The main thread just publishes targets (atomics) and reads
 *    pose snapshots (atomics).
 *  - ALWAYS call stop() in a finally block at the end of Auto.
 *
 * UNITS: X/Y in centimeters, heading in degrees (CCW positive), +X forward, +Y left.
 */
public class Drivetrain {

    // =========================================================================
    // 1. HARDWARE & CONSTANTS (TUNE THESE FOR YOUR ROBOT)
    // =========================================================================
    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private final SparkFunOTOS otos;

    // OTOS mounting offset from robot center (cm & degrees)
    private static final double OTOS_OFFSET_X_CM = 0.0;       // + is forward of center
    private static final double OTOS_OFFSET_Y_CM = 0.0;       // + is left of center
    private static final double OTOS_OFFSET_H_DEGREES = 0.0;  // angular offset if mounted rotated

    // OTOS scalar calibration
    private static final double OTOS_LINEAR_SCALAR = 1.0;
    private static final double OTOS_ANGULAR_SCALAR = 1.0;

    // Compensates for mecanum strafe slip in TeleOp field-centric driving
    private static final double LATERAL_MULTIPLIER = 1.15;

    // Autonomous PD constants (cm & degrees). MUST BE TUNED ON YOUR FIELD.
    private static final double KP_TRANSLATION = 0.018;
    private static final double KD_TRANSLATION = 0.0008;
    private static final double KP_HEADING = 0.025;
    private static final double KD_HEADING = 0.001;

    // Static-friction feedforward: smallest power that barely moves/turns the robot
    private static final double MIN_TRANSLATION_POWER = 0.08;
    private static final double MIN_HEADING_POWER = 0.06;

    // Tolerances & settling
    private static final double POSITION_TOLERANCE_CM = 2.0;
    private static final double HEADING_TOLERANCE_DEGREES = 1.5;
    private static final double SETTLE_TIME_SECONDS = 0.15;

    // Control loop timing
    private static final long CONTROL_PERIOD_MS = 20L;        // 50 Hz
    private static final double NOMINAL_DT = 0.020;
    private static final double MIN_DT = 0.005;               // clamp so derivative can't blow up
    private static final double OVERRUN_PERIOD_MS = 30.0;     // period above this counts as an overrun

    // =========================================================================
    // 2. IMMUTABLE DATA TYPES & SHARED STATE
    // =========================================================================
    /** Immutable pose snapshot (X/Y in cm, H in degrees). */
    public static final class Pose {
        public final double x, y, h;

        public Pose(double x, double y, double h) {
            this.x = x;
            this.y = y;
            this.h = h;
        }
    }

    /** Immutable autonomous setpoint. Identity (==) is used to tell targets apart. */
    public static final class Target {
        public final double x, y, headingDeg, maxSpeed;

        public Target(double x, double y, double headingDeg, double maxSpeed) {
            this.x = x;
            this.y = y;
            this.headingDeg = headingDeg;
            this.maxSpeed = maxSpeed;
        }
    }

    // Last known pose, survives between OpModes (Auto -> TeleOp) while the RC app stays alive.
    // null means "no saved pose": the next Drivetrain zeroes the OTOS.
    private static volatile Pose persistedPose = null;

    /** Call from Auto init if you want a fresh (0,0,0) start instead of restoring the last pose. */
    public static void clearSavedPose() {
        persistedPose = null;
    }

    // Cross-thread state (all atomic / volatile)
    private final AtomicReference<Pose> latestPose = new AtomicReference<>(new Pose(0, 0, 0));
    private final AtomicReference<Pose> latestVelocity = new AtomicReference<>(new Pose(0, 0, 0));
    private final AtomicReference<Target> targetRef = new AtomicReference<>(null);
    private final AtomicReference<Target> arrivedTarget = new AtomicReference<>(null);
    private final AtomicReference<Pose> pendingOdometryReset = new AtomicReference<>(null);
    private final AtomicReference<String> lastControlError = new AtomicReference<>(null);
    private volatile boolean running = false;

    // Loop stats (written only by the control thread, read for telemetry)
    private volatile double loopPeriodAvgMs = 0;
    private volatile double loopPeriodMaxMs = 0;
    private volatile double tickWorkMaxMs = 0;
    private volatile long overrunCount = 0;

    private ScheduledExecutorService controlExecutor;
    private ScheduledFuture<?> controlTask;

    // Controller state: touched ONLY by the control thread while running
    private Target lastProcessedTarget = null;
    private double prevErrorX = 0, prevErrorY = 0, prevErrorHeading = 0;
    private boolean seedDerivative = true;
    private long lastTickNano = 0L;
    private boolean isSettling = false;
    private long settleStartNano = 0L;

    // =========================================================================
    // 3. CONSTRUCTOR & HARDWARE INITIALIZATION
    // =========================================================================
    public Drivetrain(HardwareMap hardwareMap) {
        try {
            frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
            frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
            backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
            backRight = hardwareMap.get(DcMotorEx.class, "backRight");

            // Verify these on your chassis
            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            backRight.setDirection(DcMotorSimple.Direction.FORWARD);

            configureMotor(frontLeft);
            configureMotor(frontRight);
            configureMotor(backLeft);
            configureMotor(backRight);

            otos = hardwareMap.get(SparkFunOTOS.class, "otos");
            configureOtos();

        } catch (Exception e) {
            throw new RuntimeException(
                    "Drivetrain init failed: check motor/OTOS names in config. " + e.getMessage(), e);
        }
    }

    private void configureMotor(DcMotorEx motor) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /** Keep the robot completely still during this (IMU calibration). */
    private void configureOtos() {
        otos.setLinearUnit(DistanceUnit.CM);
        otos.setAngularUnit(AngleUnit.DEGREES);

        otos.setOffset(new SparkFunOTOS.Pose2D(
                OTOS_OFFSET_X_CM, OTOS_OFFSET_Y_CM, OTOS_OFFSET_H_DEGREES));
        otos.setLinearScalar(OTOS_LINEAR_SCALAR);
        otos.setAngularScalar(OTOS_ANGULAR_SCALAR);

        otos.calibrateImu();

        Pose saved = persistedPose;
        Pose start;
        if (saved == null) {
            otos.resetTracking();
            start = new Pose(0, 0, 0);
        } else {
            otos.setPosition(new SparkFunOTOS.Pose2D(saved.x, saved.y, saved.h));
            start = saved;
        }
        persistedPose = start;
        latestPose.set(start);
        latestVelocity.set(new Pose(0, 0, 0));
    }

    // =========================================================================
    // 3B. CONTROL THREAD LIFECYCLE (call from the main thread only)
    // =========================================================================
    /** Starts the 50 Hz control thread. No-op if already running. */
    public synchronized void start() {
        if (running) return;

        lastTickNano = 0L;
        lastProcessedTarget = null;
        seedDerivative = true;
        isSettling = false;
        loopPeriodAvgMs = 0;
        loopPeriodMaxMs = 0;
        tickWorkMaxMs = 0;
        overrunCount = 0;
        lastControlError.set(null);
        arrivedTarget.set(null);

        running = true;
        controlExecutor = Executors.newSingleThreadScheduledExecutor(r -> {
            Thread t = new Thread(r, "Drivetrain-Control-50Hz");
            t.setDaemon(true);
            return t;
        });
        controlTask = controlExecutor.scheduleWithFixedDelay(
                this::controlTick, 0L, CONTROL_PERIOD_MS, TimeUnit.MILLISECONDS);
    }

    /** Stops the control thread, clears the target, and zeros the motors. Safe to call repeatedly. */
    public synchronized void stop() {
        running = false;

        if (controlTask != null) {
            controlTask.cancel(false);
            controlTask = null;
        }
        if (controlExecutor != null) {
            controlExecutor.shutdownNow();
            try {
                controlExecutor.awaitTermination(250L, TimeUnit.MILLISECONDS);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            controlExecutor = null;
        }

        targetRef.set(null);
        arrivedTarget.set(null);
        pendingOdometryReset.set(null);
        zeroMotors();
    }

    // =========================================================================
    // 4. TELEOP DRIVING (main thread; ignored while the control thread is running)
    // =========================================================================
    /** Robot-centric drive. Does not touch the OTOS. */
    public void driveRobotCentric(double forward, double strafe, double turn) {
        if (running) return;

        setMotorPowers(
                forward + strafe + turn,
                forward - strafe - turn,
                forward - strafe + turn,
                forward + strafe - turn);
    }

    /** Field-centric drive. One OTOS read per call (pose only). */
    public void driveFieldCentric(double forward, double strafe, double turn) {
        if (running) return;
        readOtosHardware(false);

        double heading = Math.toRadians(latestPose.get().h);
        double cos = Math.cos(-heading);
        double sin = Math.sin(-heading);

        double rotX = strafe * cos - forward * sin;
        double rotY = strafe * sin + forward * cos;
        rotX *= LATERAL_MULTIPLIER;

        setMotorPowers(
                rotY + rotX + turn,
                rotY - rotX - turn,
                rotY - rotX + turn,
                rotY + rotX - turn);
    }

    /** Sets heading to 0 while keeping the current X/Y. */
    public void resetFieldCentricHeading() {
        Pose p = latestPose.get();
        resetOdometry(p.x, p.y, 0.0);
    }

    // =========================================================================
    // 5. AUTONOMOUS NAVIGATION
    // =========================================================================
    /**
     * Publishes a target for the control thread (starting it if needed). Call every loop.
     *
     * @return true once the robot has reached THIS target and settled; false while moving.
     */
    public boolean moveToPose(double targetX, double targetY, double targetHeadingDeg, double maxSpeed) {
        if (!running) start();

        Target current = targetRef.get();
        if (current == null
                || current.x != targetX
                || current.y != targetY
                || current.headingDeg != targetHeadingDeg
                || current.maxSpeed != maxSpeed) {
            targetRef.set(new Target(targetX, targetY, targetHeadingDeg, maxSpeed));
            return false;
        }
        // Arrival is tied to the exact Target object, so a stale arrival can never match a new target.
        return arrivedTarget.get() == current;
    }

    /** One 50 Hz tick. Sole reader/writer of the OTOS and motors while running. */
    private void controlTick() {
        if (!running) return;
        final long now = System.nanoTime();

        try {
            // ---- timing & stats ----
            double dt = NOMINAL_DT;
            if (lastTickNano != 0L) {
                double periodSec = (now - lastTickNano) * 1e-9;
                dt = Math.max(MIN_DT, periodSec);
                double periodMs = periodSec * 1e3;
                loopPeriodAvgMs = (loopPeriodAvgMs == 0) ? periodMs : 0.95 * loopPeriodAvgMs + 0.05 * periodMs;
                if (periodMs > loopPeriodMaxMs) loopPeriodMaxMs = periodMs;
                if (periodMs > OVERRUN_PERIOD_MS) overrunCount++;
            }
            lastTickNano = now;

            // ---- apply queued odometry reset, THEN read, so we always read the new pose ----
            Pose resetReq = pendingOdometryReset.getAndSet(null);
            if (resetReq != null) {
                otos.setPosition(new SparkFunOTOS.Pose2D(resetReq.x, resetReq.y, resetReq.h));
                arrivedTarget.set(null);
                isSettling = false;
                seedDerivative = true;
            }
            readOtosHardware(true);
            Pose pose = latestPose.get();

            // ---- target ----
            Target target = targetRef.get();
            if (target == null) {
                zeroMotors();
                seedDerivative = true;
                return;
            }
            if (target != lastProcessedTarget) {
                lastProcessedTarget = target;
                isSettling = false;
                seedDerivative = true;
            }

            // ---- errors ----
            double errorX = target.x - pose.x;
            double errorY = target.y - pose.y;
            double errorHeading = normalizeAngleDegrees(target.headingDeg - pose.h);

            // Seed previous error so the first derivative is zero (no kick)
            if (seedDerivative) {
                prevErrorX = errorX;
                prevErrorY = errorY;
                prevErrorHeading = errorHeading;
                seedDerivative = false;
            }

            boolean inPosition = Math.hypot(errorX, errorY) <= POSITION_TOLERANCE_CM;
            boolean inHeading = Math.abs(errorHeading) <= HEADING_TOLERANCE_DEGREES;

            // ---- arrived / settling ----
            if (inPosition && inHeading) {
                zeroMotors();
                if (!isSettling) {
                    isSettling = true;
                    settleStartNano = now;
                }
                if ((now - settleStartNano) * 1e-9 >= SETTLE_TIME_SECONDS) {
                    arrivedTarget.set(target);
                }
                // Keep prev errors current so drifting back out doesn't produce a derivative spike
                prevErrorX = errorX;
                prevErrorY = errorY;
                prevErrorHeading = errorHeading;
                return;
            }
            isSettling = false;
            arrivedTarget.compareAndSet(target, null);

            // ---- PD in field coordinates ----
            double dErrX = (errorX - prevErrorX) / dt;
            double dErrY = (errorY - prevErrorY) / dt;
            double dErrH = (errorHeading - prevErrorHeading) / dt;
            prevErrorX = errorX;
            prevErrorY = errorY;
            prevErrorHeading = errorHeading;

            double fieldX = KP_TRANSLATION * errorX + KD_TRANSLATION * dErrX;
            double fieldY = KP_TRANSLATION * errorY + KD_TRANSLATION * dErrY;
            double turn = KP_HEADING * errorHeading + KD_HEADING * dErrH;

            // ---- field -> robot frame ----
            double headingRad = Math.toRadians(pose.h);
            double cos = Math.cos(-headingRad);
            double sin = Math.sin(-headingRad);
            double robotForward = fieldX * cos - fieldY * sin;
            double robotStrafe = fieldX * sin + fieldY * cos;

            // ---- static friction feedforward & speed cap ----
            double maxSpeed = Math.max(0.0, Math.min(1.0, target.maxSpeed));

            if (inPosition) {
                robotForward = 0;
                robotStrafe = 0;
            } else {
                double mag = Math.hypot(robotForward, robotStrafe);
                double minTrans = Math.min(MIN_TRANSLATION_POWER, maxSpeed);
                if (mag > 1e-6 && mag < minTrans) {
                    robotForward *= minTrans / mag;
                    robotStrafe *= minTrans / mag;
                    mag = minTrans;
                }
                if (mag > maxSpeed && mag > 1e-6) {
                    robotForward *= maxSpeed / mag;
                    robotStrafe *= maxSpeed / mag;
                }
            }

            if (inHeading) {
                turn = 0;
            } else {
                double minTurn = Math.min(MIN_HEADING_POWER, maxSpeed);
                if (Math.abs(turn) < minTurn) turn = Math.copySign(minTurn, turn);
                turn = clip(turn, -maxSpeed, maxSpeed);
            }

            // ---- mecanum kinematics (+Y is left) ----
            setMotorPowers(
                    robotForward - robotStrafe - turn,
                    robotForward + robotStrafe + turn,
                    robotForward + robotStrafe - turn,
                    robotForward - robotStrafe + turn);

        } catch (Throwable t) {
            // Fail safe: kill motors, record the error for the main thread. No telemetry from this thread.
            try { frontLeft.setPower(0); } catch (Throwable ignored) {}
            try { frontRight.setPower(0); } catch (Throwable ignored) {}
            try { backLeft.setPower(0); } catch (Throwable ignored) {}
            try { backRight.setPower(0); } catch (Throwable ignored) {}

            StringWriter sw = new StringWriter();
            t.printStackTrace(new PrintWriter(sw));
            lastControlError.set(sw.toString());
            seedDerivative = true;
        } finally {
            double workMs = (System.nanoTime() - now) * 1e-6;
            if (workMs > tickWorkMaxMs) tickWorkMaxMs = workMs;
        }
    }

    // =========================================================================
    // 6. ODOMETRY
    // =========================================================================
    /** Refreshes pose + velocity from the OTOS. No-op while the control thread owns the sensor. */
    public void updateOdometry() {
        if (running) return;
        readOtosHardware(true);
    }

    /**
     * Reads the OTOS and publishes immutable snapshots.
     * Velocity is a second I2C transaction; skip it where you don't need it.
     * (If your SDK's SparkFunOTOS has getPosVelAcc(pos, vel, acc), use it here for one burst read.)
     */
    private void readOtosHardware(boolean includeVelocity) {
        SparkFunOTOS.Pose2D p = otos.getPosition();
        Pose pose = new Pose(p.x, p.y, p.h);
        latestPose.set(pose);
        persistedPose = pose;

        if (includeVelocity) {
            SparkFunOTOS.Pose2D v = otos.getVelocity();
            latestVelocity.set(new Pose(v.x, v.y, v.h));
        }
    }

    /**
     * Sets the robot's known pose (cm, degrees). If the control thread is running, the write is
     * queued and applied by that thread on its next tick; getPose() updates within ~20 ms.
     */
    public void resetOdometry(double startX, double startY, double startHeadingDeg) {
        Pose p = new Pose(startX, startY, startHeadingDeg);

        if (running) {
            pendingOdometryReset.set(p);
        } else {
            otos.setPosition(new SparkFunOTOS.Pose2D(startX, startY, startHeadingDeg));
            latestPose.set(p);
            latestVelocity.set(new Pose(0, 0, 0));
            persistedPose = p;
            arrivedTarget.set(null);
        }
    }

    // =========================================================================
    // 7. HELPERS & GETTERS
    // =========================================================================
    /** Scales all four powers down together if any exceeds 1.0. */
    private void setMotorPowers(double fl, double fr, double bl, double br) {
        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    private void zeroMotors() {
        setMotorPowers(0, 0, 0, 0);
    }

    private static double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double normalizeAngleDegrees(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }

    // Pose (cm, degrees)
    public Pose getPose() { return latestPose.get(); }
    public double getX() { return latestPose.get().x; }
    public double getY() { return latestPose.get().y; }
    public double getHeadingDegrees() { return latestPose.get().h; }
    public double getHeadingRadians() { return Math.toRadians(latestPose.get().h); }

    // Velocity (cm/s, deg/s)
    public double getVelocityX() { return latestVelocity.get().x; }
    public double getVelocityY() { return latestVelocity.get().y; }
    public double getSpeed() {
        Pose v = latestVelocity.get();
        return Math.hypot(v.x, v.y);
    }
    public double getAngularVelocityDegrees() { return latestVelocity.get().h; }

    // Diagnostics: put these in telemetry from the MAIN thread
    public boolean isControlThreadRunning() { return running; }
    public double getLoopPeriodAvgMs() { return loopPeriodAvgMs; }   // should sit at ~20.0
    public double getLoopPeriodMaxMs() { return loopPeriodMaxMs; }
    public double getTickWorkMaxMs() { return tickWorkMaxMs; }       // must stay well under 20
    public long getOverrunCount() { return overrunCount; }
    public String getLastControlError() { return lastControlError.get(); }
}