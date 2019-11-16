package org.firstinspires.ftc.teamcode.lib;

import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;

public class PidfController {

    private final DoubleSupplier P;
    private final DoubleSupplier I;
    private final DoubleSupplier D;
    private final DoubleUnaryOperator F;

    private double minOut;
    private double maxOut;

    private boolean running;
    private double iSum;
    private double lastErr;
    private long lastReadNanos;
    private static final double NANOS_PER_SEC = 1_000_000_000;

    public PidfController(double P, double I, double D, double F) {
        this(P, I, D, (pos) -> F);
    }

    public PidfController(double P, double I, double D, DoubleUnaryOperator F) {
        this(() -> P, () -> I, () -> D, F);
    }

    public PidfController(DoubleSupplier P, DoubleSupplier I, DoubleSupplier D, DoubleUnaryOperator F) {
        this.P = P;
        this.I = I;
        this.D = D;
        this.F = F;
        minOut = -1;
        maxOut = 1;
        running = false;
    }

    public void setOutputRange(double min, double max) {
        minOut = min;
        maxOut = max;
    }

    public void start() {
        lastErr = Double.NaN;
        lastReadNanos = System.nanoTime();
        iSum = 0;
        running = true;
    }

    public void stop() {
        running = false;
    }

    public double getOutput(double position, double target) {
        if (!running) {
            throw new IllegalStateException("Pid controller must have been started to be used.");
        }

        double error = target - position;

        long now = System.nanoTime();
        double elapsedTime = (now - lastReadNanos) / NANOS_PER_SEC;
        lastReadNanos = now;

        iSum += elapsedTime * error;
        double deltaErr = Double.isNaN(lastErr) ? 0 : (error - lastErr) / elapsedTime;
        lastErr = error;

        double p = P.getAsDouble() * error;
        double i = I.getAsDouble() * iSum;
        double d = D.getAsDouble() * deltaErr;
        double f = F.applyAsDouble(position);

        iSum = bound(minOut, iSum, maxOut);

        return bound(minOut, p + i + d + f, maxOut);
    }

    private double bound(double min, double n, double max) {
        return n < min ? min : n > max ? max : n;
    }

}
