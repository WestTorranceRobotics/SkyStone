package org.westtorrancerobotics.lib.spline;

import org.westtorrancerobotics.lib.spline.geom.Point;
import java.util.Objects;
import org.westtorrancerobotics.lib.functionmath.Polynomial;
import org.westtorrancerobotics.lib.functionmath.interfaces.CalculusFunction;
import org.westtorrancerobotics.lib.functionmath.Piecewise;
import org.westtorrancerobotics.lib.functionmath.PolynomialGenerator;

/**
 * A function limited by the maximums and minimums of its first and second
 * derivatives. Because the output is assumed to be a velocity, the first derivative
 * assumes the name acceleration, and the second assumes the name jerk, despite their
 * conventional name assignments when an output is position. The function created
 * is limited by derivatives, and uses its initial output, end point, and a target
 * output to maintain in between in constructing itself.
 * 
 * @since 1.1
 */
public class VelocitySpline implements CalculusFunction {
    
    private final CalculusFunction spline;
    private final double totalTime;

    /**
     * Creates a function limited by the maximums and minimums of its first and second
     * derivatives with domain [0, {@code length}]. The function will be such that
     * f(0) = {@code vinit}, f({@code length}) = {@code vfinal}, and for as many inputs
     * n as are possible, f(n) = {@code vmid}. The range of f'(x) at any x in the
     * domain [0, {@code length}] will be included in [-{@code MAX_ACCELERATION},
     * {@code MAX_ACCELERATION}] and the range of f''(x) at any x in the domain
     * [0, {@code length}] will be included in [-{@code MAX_JERK}, {@code MAX_JERK}].
     * If this is impossible, a linear function including the points (0, {@code vinit})
     * and ({@code length}, {@code vfinal}) will be returned. If {@code length}
     * is zero, an {@link ArithmeticException} will be thrown. The function is not
     * guaranteed to have exact results if {@code length} is too small for more
     * than a single input to exist f(n) = {@code vmid}, and {@code ACCURACY}
     * makes the function determine its outputs more precisely with a smaller positive
     * value. Additional calculations take place only during initialization.
     * 
     * @param vinit output desired in the constructed function at x = 0
     * @param vmid output desired in the constructed function for as many input values
     *             as possible
     * @param vfinal output desired in the constructed function at x = {@code length}
     * @param length specification of the function's valid domain and of the input at
     *               which {@code vfinal must output}
     * @param MAX_ACCELERATION limit of the function's first derivative's magnitude
     * @param MAX_JERK limit of the function's second derivative's magnitude
     * @param ACCURACY specification of calculation error tolerance
     * @since 1.1
     */
    public VelocitySpline(double dst, double vel, double acc, double jerk) {
        boolean a2Gj = acc*acc < jerk * vel;
        double isdx = a2Gj ? 2*acc/jerk + (vel - acc*acc/jerk) / acc : 2*Math.sqrt(vel/jerk);
        double t;
        Point[] points;
        if (isdx * vel < dst) {
            t = (dst - vel * isdx) / vel + 2 * isdx;
            if (a2Gj) {
                points = new Point[]{
                    new Point(0, 0),
                    new Point(acc/jerk, acc),
                    new Point(isdx - acc/jerk, acc),
                    new Point(isdx, 0),
                    new Point(t - isdx, 0),
                    new Point(t - isdx + acc/jerk, -acc),
                    new Point(t - acc/jerk, -acc),
                    new Point(t, 0)
                };
            } else {
                points = new Point[]{
                    new Point(0, 0),
                    new Point(isdx/2, jerk*isdx/2),
                    new Point(isdx, 0),
                    new Point(t - isdx, 0),
                    new Point(t - isdx/2, -jerk*isdx/2),
                    new Point(t, 0)
                };
            }
        } else {
            if (dst < 2 * (acc*acc*acc)/(jerk*jerk)) {
                t = Math.cbrt(32 * dst / jerk);
            } else {
                double a = acc / 4;
                double b = -(acc*acc)/(2*jerk);
                double c = -dst;
                t = (b + Math.sqrt(b*b - 4*a*c)) / (2*a);
            }
            if (4*acc/jerk > t) {
                points = new Point[]{
                    new Point(0, 0),
                    new Point(t/4, jerk * t/4),
                    new Point(3*t/4, -jerk * t/4),
                    new Point(t, 0)
                };
            } else {
                points = new Point[]{
                    new Point(0, 0),
                    new Point(acc/jerk, acc),
                    new Point(t/2 - acc/jerk, acc),
                    new Point(t/2 + acc/jerk, -acc),
                    new Point(t - acc/jerk, -acc),
                    new Point(t, 0)
                };
            }
        }
        Polynomial[] lines = new Polynomial[points.length + 1];
        double[] bounds = new double[points.length + 2];
        for (int i = 1; i < lines.length - 1; i++) {
            bounds[i] = points[i - 1].x;
            lines[i] = PolynomialGenerator.generateLine(points[i - 1], points[i]);
        }
        lines[0] = new Polynomial(0);
        lines[lines.length - 1] = new Polynomial(0);
        bounds[0] = Double.NEGATIVE_INFINITY;
        bounds[lines.length - 1] = points[points.length - 1].x;
        bounds[lines.length] = Double.POSITIVE_INFINITY;
        totalTime = points[points.length - 1].x;
        this.spline = new Piecewise(lines, bounds).integral();
    }
    
    public double getTotalTime() {
        return totalTime;
    }

    /**
     * 
     * @see #VelocitySpline(double, double, double, double, double, double, double)
     * @since 1.1
     */
    @Override
    public double get(double x) {
        return spline.get(x);
    }

    /**
     * 
     * @since 1.1
     */
    @Override
    public CalculusFunction derivative() {
        return spline.derivative();
    }

    /**
     * 
     * @since 1.1
     */
    @Override
    public CalculusFunction integral() {
        return spline.integral();
    }

    /**
     * 
     * @since 1.1
     */
    @Override
    public String toString() {
        return spline.toString();
    }
    
    /**
     * 
     * @since 1.1
     */
    @Override
    public boolean equals(Object obj) {
        if (obj == null) {
            return this == null;
        }
        if (!(obj.getClass().equals(getClass()))) {
            return false;
        }
        VelocitySpline vsp = (VelocitySpline) obj;
        return vsp.spline.equals(spline);
    }

    /**
     * 
     * @since 1.1
     */
    @Override
    public int hashCode() {
        int hash = 3;
        hash = 23 * hash + Objects.hashCode(this.spline);
        return hash;
    }
}