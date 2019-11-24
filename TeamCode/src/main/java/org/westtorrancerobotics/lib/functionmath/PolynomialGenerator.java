package org.westtorrancerobotics.lib.functionmath;

import org.westtorrancerobotics.lib.spline.geom.Point;
import org.westtorrancerobotics.lib.functionmath.interfaces.DerivableFunction;

/**
 * Utility class used for creating polynomials from points and functions, rather
 * than coefficients.
 * 
 * @since 1.0
 */
public class PolynomialGenerator {
    
    private PolynomialGenerator() {} // no constructor
    
    /**
     * Creates a line connecting two points. The first two parameters are the x and
     * y of the first point. The second two parameters are the x and y of the second
     * point. If there is no function connecting the two points, or if the points
     * are the same, an exception will be thrown.
     * 
     * @param x1 the x of the first point
     * @param y1 the y of the first point
     * @param x2 the x of the second point
     * @param y2 the y of the second point
     * @return a line connecting the points
     * @throws ArithmeticException if the points have the same x value
     * @since 1.0
     */
    public static Polynomial generateLine(double x1, double y1, double x2, double y2) {
        if (x1 == x2) {
            throw new ArithmeticException("Equal x values: " + x2);
        }
        double m = (y2 - y1) / (x2 - x1);
        return new Polynomial(m, y1 - m * x1);
    }
    
    /**
     * Creates a line connecting two points. If there is no function connecting
     * the two points, or if the points are the same, an exception will be thrown.
     * 
     * @param i the first point
     * @param f the second point
     * @return a line connecting the points
     * @throws ArithmeticException if the points have the same x value
     * @since 1.0
     */
    
    public  static Polynomial generateLine(Point i, Point f) {
        return generateLine(i.x, i.y, f.x, f.y);
    }
    
    /**
     * Creates a parabola through two points and with a specific second degree coefficient.
     * 
     * @param a the coefficient of the quadratic term of the output polynomial
     * @param x1 the x of the first point through which the parabola will travel
     * @param y1 the y of the first point through which the parabola will travel
     * @param x2 the x of the second point through which the parabola will travel
     * @param y2 the y of the second point through which the parabola will travel
     * @return a parabola that travels through the points and has the specified a value
     * @throws ArithmeticException if the x values are the same
     * @since 1.0
     */
    public static Polynomial generateParabola(double a, double x1, double y1, double x2, double y2) {
        if (x1 == x2) {
            throw new ArithmeticException(String.format("Need two distinct x values, supplied: %f, %f.", x1, x2));
        }
        double b = (y1-a*x1*x1-y2+a*x2*x2)/(x1-x2);
        double c = y1-a*x1*x1-b*x1;
        Polynomial p = new Polynomial(a, b, c);
        return p;
    }
    
    /**
     * Creates a parabola through two points and with a specific second degree coefficient.
     * 
     * @param a the coefficient of the quadratic term of the output polynomial
     * @param i the first point through which the parabola will travel
     * @param f the second point through which the parabola will travel
     * @return a parabola that travels through the points and has the specified a value
     * @throws ArithmeticException if the x values are the same
     * @since 1.0
     */
    public static Polynomial generateParabola(double a, Point i, Point f) {
        return generateParabola(a, i.x, i.y, f.x, f.y);
    }
    
    /**
     * Creates a parabola that goes through three points.
     * 
     * @param x1 the x coordinate of the first point
     * @param y1 the y coordinate of the first point
     * @param x2 the x coordinate of the second point
     * @param y2 the y coordinate of the second point
     * @param x3 the x coordinate of the third point
     * @param y3 the y coordinate of the third point
     * @return a parabola connecting the three points
     * @throws ArithmeticException if there are multiple points with the same x
     *                             coordinates
     * @since 1.0
     */
    public static Polynomial generateParabola(double x1, double y1, double x2, double y2, double x3, double y3) {
        if (x1 == x2 || x2 == x3 || x1 == x3) {
            throw new ArithmeticException(String.format("Need three distinct x values, supplied: %f, %f, %f.", x1, x2, x3));
        }
        else {
            double a = (y1*x2+x1*y3+y2*x3-y3*x2-x3*y1-y2*x1)
                            /(x1*x1*x2+x3*x3*x1+x3*x2*x2-x3*x3*x2-x3*x1*x1-x2*x2*x1);
            double b = (x1*x1*y2+y1*x3*x3+x2*x2*y3-x3*x3*y2-x1*x1*y3-x2*x2*y1)
                            /(x1*x1*x2+x3*x3*x1+x3*x2*x2-x3*x3*x2-x3*x1*x1-x2*x2*x1);
            double c = (x1*x1*x2*y3+x1*y2*x3*x3+y1*x3*x2*x2-x3*x3*x2*y1-x3*y2*x1*x1-y3*x1*x2*x2)
                            /(x1*x1*x2+x3*x3*x1+x3*x2*x2-x3*x3*x2-x3*x1*x1-x2*x2*x1);
            if (!Double.isFinite(a) || !Double.isFinite(b) || !Double.isFinite(c)) {
                throw new ArithmeticException("Math failure");
            }
            Polynomial p = new Polynomial(a, b, c);
            return p;
        }
    }
    
    /**
     * Creates a parabola that goes through three points.
     * 
     * @param a the first point
     * @param b second point
     * @param c the third point
     * @return a parabola connecting the three points
     * @throws ArithmeticException if there are multiple points with the same x
     *                             coordinates
     * @since 1.0
     */
    public static Polynomial generateParabola(Point a, Point b, Point c) {
        return generateParabola(a.x, a.y, b.x, b.y, c.x, c.y);
    }
    
    /**
     * Creates a cubic polynomial that goes through four points.
     * 
     * @param x1 the x coordinate of the first point
     * @param y1 the y coordinate of the first point
     * @param x2 the x coordinate of the second point
     * @param y2 the y coordinate of the second point
     * @param x3 the x coordinate of the third point
     * @param y3 the y coordinate of the third point
     * @param x4 the x coordinate of the fourth point
     * @param y4 the y coordinate of the fourth point
     * @return a cubic polynomial connecting the four points
     * @throws ArithmeticException if there are multiple points with the same x
     *                             coordinates
     * @since 1.0
     */
    public static Polynomial generateCubic(
            double x1, double y1,
            double x2, double y2,
            double x3, double y3,
            double x4, double y4) {
        if (x1 == x2 || x1 == x3 || x1 == x4 || x2 == x3 || x2 == x4 || x3 == x4) {
            throw new ArithmeticException(String.format(
                    "Need four distinct x values, supplied: %f, %f, %f, %f.", x1, x2, x3, x4));
        }
        double q1 = y1 / ((x1 - x2) * (x1 - x3) * (x1 - x4));
        double q2 = y2 / ((x2 - x1) * (x2 - x3) * (x2 - x4));
        double q3 = y3 / ((x3 - x1) * (x3 - x2) * (x3 - x4));
        double q4 = y4 / ((x4 - x1) * (x4 - x2) * (x4 - x3));
        return new Polynomial(
                q1+q2+q3+q4,
                -(q1*(x2+x3+x4)+q2*(x1+x3+x4)+q3*(x1+x2+x4)+q4*(x1+x2+x3)),
                q1*(x2*x3+x2*x4+x3*x4)+q2*(x1*x3+x1*x4+x3*x4)+q3*(x1*x2+x1*x4+x2*x4)+q4*(x1*x2+x1*x3+x2*x3),
                -(q1*x2*x3*x4+q2*x1*x3*x4+q3*x1*x2*x4+q4*x1*x2*x3)
        );
    }
    
    /**
     * Creates a cubic polynomial that goes through four points.
     * 
     * @param a the first point
     * @param b second point
     * @param c the third point
     * @param d the fourth point
     * @return a cubic polynomial connecting the four points
     * @throws ArithmeticException if there are multiple points with the same x
     *                             coordinates
     * @since 1.0
     */
    public static Polynomial generateCubic(Point a, Point b, Point c, Point d) {
        return generateCubic(a.x, a.y, b.x, b.y, c.x, c.y, d.x, d.y);
    }
    
    /**
     * Gives the Taylor series approximation of the function for some number of terms.
     * The point from which the series will search derivatives and for which the output
     * of the approximation and original will be identical is the input value of
     * zero.
     * 
     * @param f the function to approximate
     * @param degree the number of terms to use in the expansion
     * @return a Taylor series approximation of the function
     * @throws IllegalArgumentException if the degree specified is less than zero
     * @since 1.0
     */
    public static Polynomial generateTaylorSeries (DerivableFunction f, int degree) {
        if (degree < 0) {
            throw new IllegalArgumentException("Must have nonnegative degree: " + degree);
        }
        double[] outCoefficients = new double[degree + 1];
        double denominator = 1;
        for (int myDegree = 0; myDegree <= degree; ) {
            double numerator = f.get(0);
            double quotient = numerator / denominator;
            outCoefficients[degree - myDegree] = quotient;
            denominator *= ++myDegree;
            f = f.derivative();
        }
        return new Polynomial(outCoefficients);
    }
}
