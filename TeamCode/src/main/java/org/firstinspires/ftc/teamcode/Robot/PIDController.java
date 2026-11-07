package org.firstinspires.ftc.teamcode.Robot;

public class PIDController {
    private final double kP, kI, kD, maxOut;
    private double integral = 0, lastError = 0;
    private long lastNanos = 0;

    public PIDController(double kP, double kI, double kD, double maxOut) {
        this.kP = kP; this.kI = kI; this.kD = kD; this.maxOut = maxOut;
    }

    public void reset() {
        integral = 0;
        lastError = 0;
        lastNanos = 0;
    }

    public double calculate(double error) {
        long now = System.nanoTime();
        double dt = (lastNanos == 0) ? 0.02 : (now - lastNanos) / 1e9;
        lastNanos = now;
        dt = Math.max(dt, 1e-3);

        integral = Math.max(-0.2, Math.min(0.2, integral + error * dt)); // anti-windup
        double derivative = (error - lastError) / dt;
        lastError = error;

        double out = kP * error + kI * integral + kD * derivative;
        return Math.max(-maxOut, Math.min(maxOut, out));
    }
}