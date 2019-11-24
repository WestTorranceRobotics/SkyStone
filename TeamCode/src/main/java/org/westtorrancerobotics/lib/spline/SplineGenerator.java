package org.westtorrancerobotics.lib.spline;

import org.westtorrancerobotics.lib.spline.geom.Location;
import org.westtorrancerobotics.lib.spline.geom.Point;
import org.westtorrancerobotics.lib.spline.geom.Angle;
import org.westtorrancerobotics.lib.functionmath.ParametricFunction;
import org.westtorrancerobotics.lib.functionmath.Sum;
import org.westtorrancerobotics.lib.functionmath.Polynomial;
import org.westtorrancerobotics.lib.functionmath.Piecewise;
import org.westtorrancerobotics.lib.functionmath.Product;
import org.westtorrancerobotics.lib.functionmath.Constant;
import org.westtorrancerobotics.lib.functionmath.interfaces.CalculusFunction;
import org.westtorrancerobotics.lib.functionmath.interfaces.DerivableFunction;
import org.westtorrancerobotics.lib.util.MathUtils;

/**
 * Class for creation of parametric functions to interpolate two points. Contains
 * the single {@code static} method {@link #makeSpline(Location, double, Location, double)}.
 * 
 * @see #makeSpline
 * @since 1.0
 */
public class SplineGenerator {
    
    private SplineGenerator() {} // no constructor
    
    /**
     * Creates a parametric function that begins at the specified {@code Location start}
     * and ends at the specified {@code Location end}. The path created will go from
     * looking forward (toward the spline curve, if the spline generated goes forward,
     * otherwise away from the spline curve) at the position specified by {@code start}
     * to looking forward (away the spline curve, if the spline generated goes forward,
     * otherwise toward from the spline curve) at the position specified by (@code end).
     * The start scale determines for how long the direction of {@code Location start}
     * will take precedent over the movement to the end point, and the same is true
     * for end. The initial parameter of the function generated will be zero, and
     * the final parameter of the function (at which point the output is the end location)
     * will be one.
     * <ul>
     * <li>If both scalars are positive, the path will go forward from the start to
     * the end.
     * <li>If both scalars are negative, the path will go backwards from the start
     * to the end, with the actual direction of the spline opposite what is expected.
     * <li>If one scalar is positive, and the other negative, an {@link IllegalArgumentException}
     * will be thrown.
     * <li>If one scalar is zero, the direction on that location is ignored by the path,
     * and the direction traveled by the spline will be as indicated by the other
     * scalar.
     * <li>If both scalars are zero, the spline will be assigned to travel forward,
     * arbitrarily, and the path will be a straight line from one point to the other.
     * It does not typically make sense for zero scalars to be supplied on both ends.
     * </ul>
     * A visual explanation of the use of the parameters of this function and their
     * effects on its output can be seen here: 
     * <a href="https://www.desmos.com/calculator/tavw7n71pk">Spline Stuff Public
     * on Desmos Graphing Calculator</a>.
     * 
     * @param start the first location of the parametric generated
     * @param startScale the emphasis placed on the direction of the first location
     *                   when generating the spline
     * @param end the last location of the parametric generated
     * @param endScale the emphasis placed on the direction of the last location
     *                 when generating the spline
     * @return a parametric function interpolating the two points and their slopes
     * @throws IllegalArgumentException if one scalar is positive and the other negative
     * @see ParametricFunction
     * @see Location
     * @since 1.0
     */
    public static ParametricFunction makeSpline(Location start, double startScale, Location end, double endScale) {
        if ((startScale > 0 && endScale < 0) || (startScale < 0 && endScale > 0)) {
            throw new IllegalArgumentException("Spline must either go forward or backward, not both.");
        }
        boolean forward = !(startScale + endScale < 0);
        // s1(x) in desmos
        CalculusFunction endScalar = new Piecewise(new CalculusFunction[]{
            new Polynomial(2, 0, 0),
            new Polynomial(-2, 4, -1)
        }, new double[]{Double.NEGATIVE_INFINITY, 0.5}, Double.POSITIVE_INFINITY);
        // s0(x) in desmos
        CalculusFunction startScalar = new Piecewise(new CalculusFunction[]{
            new Polynomial(-2, 0, 1),
            new Polynomial(2, -4, 2)
        }, new double[]{Double.NEGATIVE_INFINITY, 0.5}, Double.POSITIVE_INFINITY);
        //\theta_x(\theta_0) and \theta_y(\theta_0)
        Point startDir = start.direction.toRect(1);
        //\theta_x(\theta_1) and \theta_y(\theta_1)
        Point endDir = end.direction.toRect(1);
        // h in desmos
        DerivableFunction x = new Sum.Derivable(
                new Product(startScalar, new Sum(
                        new Constant(start.x),
                        new Polynomial(startDir.x * startScale, 0)
                )),
                new Product(endScalar, new Sum(
                        new Constant(end.x),
                        new Polynomial(endScale * endDir.x, -1 * endScale * endDir.x)
                ))
        );
        // k in desmos
        DerivableFunction y = new Sum.Derivable(
                new Product(startScalar, new Sum(
                        new Constant(start.y),
                        new Polynomial(startDir.y * startScale, 0)
                )),
                new Product(endScalar, new Sum(
                        new Constant(end.y),
                        new Polynomial(endScale * endDir.y, -1 * endScale * endDir.y)
                ))
        );
        //create parametric function (x, y) with t in domain [0, 1], and store direction
        return new ParametricFunction(x, y, 1, forward);
    }
    
    public static ParametricFunction makeSpline(Location start, Location end) {
        double dx = end.x - start.x;
        double dy = end.y - start.y;
        double dst = Math.sqrt(dx*dx + dy*dy);
        double dtx0 = start.direction.getX() - dx/dst;
        double dty0 = start.direction.getY() - dy/dst;
        double dtx1 = end.direction.getX() - dx/dst;
        double dty1 = end.direction.getY() - dy/dst;
        double p0 = 0.5 * (dst+Math.sqrt(dtx0*dtx0+dty0*dty0));
        double p1 = 0.5 * (dst+Math.sqrt(dtx1*dtx1+dty1*dty1));
        return makeSpline(start, p0, end, p1);
    }
    
    public static CalculusFunction generateQuinticInterpolaterFunction(
            double initSlp, double finalSlp, Point... locations) {
        final int numSegs = locations.length - 1;
        double[][] coeffs = new double[6*numSegs][6*numSegs+1];
        int nextFree = 0;
        for (int i = 0; i < numSegs; i++) {
            double[] row0 = new double[6*numSegs+1];
            double pow = 1;
            for (int j = row0.length - 2; j >= 0; j--) {
                if (j % numSegs == i) {
                    row0[j] = pow;
                    pow *= locations[i].x;
                } else {
                    row0[j] = 0;
                }
            }
            row0[row0.length - 1] = locations[i].y;
            coeffs[nextFree] = row0;
            nextFree++;
            double[] row1 = new double[6*numSegs+1];
            pow = 1;
            for (int j = row1.length - 2; j >= 0; j--) {
                if (j % numSegs == i) {
                    row1[j] = pow;
                    pow *= locations[i+1].x;
                } else {
                    row1[j] = 0;
                }
            }
            row1[row1.length - 1] = locations[i+1].y;
            coeffs[nextFree] = row1;
            nextFree++;
            if (i == 0) {
                // connect to start
                double[] row2 = new double[6*numSegs+1];
                pow = Double.NaN;
                double der = 0;
                for (int j = row2.length - 2; j >= 0; j--) {
                    if (j % numSegs == i) {
                        row2[j] = pow * der;
                        if (Double.isNaN(row2[j])) {
                            row2[j] = 0;
                        }
                        pow = Double.isNaN(pow) ? 1 : pow * locations[i].x;
                        der++;
                    } else {
                        row2[j] = 0;
                    }
                }
                row2[row2.length - 1] = initSlp;
                coeffs[nextFree] = row2;
                nextFree++;
                double[] row3 = new double[6*numSegs+1];
                pow = 1;
                double[] ders = {0, 0, 2, 6, 12, 20};
                int ind = 0;
                for (int j = row3.length - 2; j >= 0; j--) {
                    if (j % numSegs == i) {
                        if (ders[ind] == 0) {
                            ind++;
                            continue;
                        }
                        row3[j] = pow * ders[ind];
                        pow *= locations[i].x;
                        ind++;
                    } else {
                        row3[j] = 0;
                    }
                }
                row3[row3.length - 1] = 0;
                coeffs[nextFree] = row3;
                nextFree++;
            }
            if (i == numSegs - 1) {
                // connect to end
                double[] row2 = new double[6*numSegs+1];
                pow = Double.NaN;
                double der = 0;
                for (int j = row2.length - 2; j >= 0; j--) {
                    if (j % numSegs == i) {
                        row2[j] = pow * der;
                        if (Double.isNaN(row2[j])) {
                            row2[j] = 0;
                        }
                        pow = Double.isNaN(pow) ? 1 : pow * locations[i+1].x;
                        der++;
                    } else {
                        row2[j] = 0;
                    }
                }
                row2[row2.length - 1] = finalSlp;
                coeffs[nextFree] = row2;
                nextFree++;
                double[] row3 = new double[6*numSegs+1];
                pow = 1;
                double[] ders = {0, 0, 2, 6, 12, 20};
                int ind = 0;
                for (int j = row3.length - 2; j >= 0; j--) {
                    if (j % numSegs == i) {
                        if (ders[ind] == 0) {
                            ind++;
                            continue;
                        }
                        row3[j] = pow * ders[ind];
                        pow *= locations[i+1].x;
                        ind++;
                    } else {
                        row3[j] = 0;
                    }
                }
                row3[row3.length - 1] = 0;
                coeffs[nextFree] = row3;
                nextFree++;
            }
            if (i != numSegs - 1) {
                // connect to next segment
                double[] d1 = derivative(1, locations[i+1].x);
                double[] d2 = derivative(2, locations[i+1].x);
                double[] d3 = derivative(3, locations[i+1].x);
                double[] d4 = derivative(4, locations[i+1].x);
                double[] row_1 = new double[6*numSegs+1];
                double[] row_2 = new double[6*numSegs+1];
                double[] row_3 = new double[6*numSegs+1];
                double[] row_4 = new double[6*numSegs+1];
                int n = 0;
                for (int j = 0; j < 6*numSegs; j++) {
                    row_1[j] = 0;
                    row_2[j] = 0;
                    row_3[j] = 0;
                    row_4[j] = 0;
                    if (j%numSegs == i) {
                        row_1[j] = d1[n];
                        row_2[j] = d2[n];
                        row_3[j] = d3[n];
                        row_4[j] = d4[n];
                    }
                    if (j%numSegs == i + 1) {
                        row_1[j] = -d1[n];
                        row_2[j] = -d2[n];
                        row_3[j] = -d3[n];
                        row_4[j] = -d4[n];
                        n++;
                    }
                }
                coeffs[nextFree] = row_1;
                nextFree++;
                coeffs[nextFree] = row_2;
                nextFree++;
                coeffs[nextFree] = row_3;
                nextFree++;
                coeffs[nextFree] = row_4;
                nextFree++;
            }
        }
        double[] solutions = MathUtils.solveAugmentedMatrix(coeffs);
        Polynomial[] segments = new Polynomial[numSegs];
        for (int i = 0; i < numSegs; i++) {
            double[] myCoeffs = new double[6];
            for (int j = 0; j < 6; j++) {
                myCoeffs[j] = solutions[numSegs*j + i];
            }
            segments[i] = new Polynomial(myCoeffs);
        }
        double[] xs = new double[numSegs + 1];
        for (int i = 0; i < xs.length; i++) {
            xs[i] = locations[i].x;
        }
        return new Piecewise(segments, xs);
    }

    private static double[] derivative(int derivative, double x) {
        double[] zero = new double[6];
        double pow = 1;
        for (int i = 5 - derivative; i >= 0; i--) {
            zero[i] = pow;
            pow *= x;
        }
        double[][] fact = new double[][]{
            {5,4,3,2,1,0},
            {4,3,2,1,0,0},
            {3,2,1,0,0,0},
            {2,1,0,0,0,0}
        };
        for (int i = 0; i < derivative; i++) {
            for (int j = 0; j < 6; j++) {
                zero[j] *= fact[i][j];
            }
        }
        return zero;
    }
    
    public static ParametricFunction generateQuinticSpline(Angle initDir, Angle finalDir,
            double slopeBloatingConstant, Point... locations) {
        Point[] xlocations = new Point[locations.length];
        for (int i = 0; i < locations.length; i++) {
            xlocations[i] = new Point(i*slopeBloatingConstant, locations[i].x);
        }
        Point[] ylocations = new Point[locations.length];
        for (int i = 0; i < locations.length; i++) {
            ylocations[i] = new Point(i*slopeBloatingConstant, locations[i].y);
        }
        CalculusFunction x = generateQuinticInterpolaterFunction(initDir.getX(), finalDir.getX(), xlocations);
        CalculusFunction y = generateQuinticInterpolaterFunction(initDir.getY(), finalDir.getY(), ylocations);
        return new ParametricFunction(x, y, slopeBloatingConstant * (locations.length - 1));
    }
    
    public static ParametricFunction generateQuinticSpline(Angle initDir, Angle finalDir,
            Point... locations) {
        double[] dsts = new double[locations.length];
        dsts[0] = 0;
        for (int i = 1; i < dsts.length; i++) {
            dsts[i] = dsts[i - 1] + locations[i].distance(locations[i - 1]);
        }
        Point[] xlocations = new Point[locations.length];
        for (int i = 0; i < locations.length; i++) {
            xlocations[i] = new Point(dsts[i], locations[i].x);
        }
        Point[] ylocations = new Point[locations.length];
        for (int i = 0; i < locations.length; i++) {
            ylocations[i] = new Point(dsts[i], locations[i].y);
        }
        CalculusFunction x = generateQuinticInterpolaterFunction(initDir.getX(), finalDir.getX(), xlocations);
        CalculusFunction y = generateQuinticInterpolaterFunction(initDir.getY(), finalDir.getY(), ylocations);
        return new ParametricFunction(x, y, dsts[locations.length - 1]);
    }
    
}
