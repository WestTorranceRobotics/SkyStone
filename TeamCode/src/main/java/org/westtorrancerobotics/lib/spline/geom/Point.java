package org.westtorrancerobotics.lib.spline.geom;

import org.westtorrancerobotics.lib.util.StringUtils;

/**
 * The {@code Point} class defines a point representing a location
 * in {@code (x,y)} coordinate space with double precision. It contains an x and
 * a y coordinate, and methods to manipulate those coordinates.
 * 
 * @since 1.0
 * @version 1.1
 */
public class Point {

    /**
     * The x coordinate of this {@code Point}.
     * 
     * @since 1.0
     */
    public double x;
    
    /**
     * The y coordinate of this {@code Point}.
     * 
     * @since 1.0
     */
    public double y;

    /**
     * Constructs and initializes a {@code Point2D} with the specified coordinates.
     * 
     * @param x the x coordinate of the newly constructed {@code Point2D}
     * @param y the y coordinate of the newly constructed {@code Point2D}
     * @since 1.0
     */
    public Point (double x, double y) {
        this.x = x;
        this.y = y;
    }
    
    public Point (double r, Angle theta) {
        this.x = r * theta.getX();
        this.y = r * theta.getY();
    }
    
    /**
     * Translates this point, at location {@code (x,y)},
     * by {@code dx} along the {@code x} axis and {@code dy}
     * along the {@code y} axis so that it now represents the point
     * {@code (x+dx,y+dy)}.
     *
     * @param dx the distance to move this point along the x axis
     * @param dy the distance to move this point along the y axis
     * @see java.awt.Point#translate
     * @since 1.0
     */
    public void translate(double dx, double dy) {
        this.x += dx;
        this.y += dy;
    }
    
    public void translate(Point d) {
        translate(d.x, d.y);
    }

    /**
     * Returns the X coordinate of this <code>Point</code> in
     * <code>double</code> precision.
     * @return the X coordinate of this <code>Point</code>.
     * @since 1.0
     */
    public double getX() {
        return x;
    }
    
    /**
     * Returns the Y coordinate of this <code>Point</code> in
     * <code>double</code> precision.
     * @return the Y coordinate of this <code>Point</code>.
     * @since 1.0
     */
    public double getY() {
        return y;
    }

    /**
     * Sets the location of this <code>Point</code> to the
     * specified <code>double</code> coordinates.
     *
     * @param x the new X coordinate of this {@code Point}
     * @param y the new Y coordinate of this {@code Point}
     * @since 1.0
     */
    public void setLocation(double x, double y) {
        this.x = x;
        this.y = y;
    }
    
    /**
     * Sets the location of this <code>Point</code> to the same
     * coordinates as the specified <code>Point</code> object.
     * @param p the specified <code>Point</code> to which to set
     * this <code>Point</code>
     * @since 1.0
     */
    public void setLocation(Point p) {
        setLocation(p.getX(), p.getY());
    }

    /**
     * Returns the square of the distance between two points.
     *
     * @param x1 the X coordinate of the first specified point
     * @param y1 the Y coordinate of the first specified point
     * @param x2 the X coordinate of the second specified point
     * @param y2 the Y coordinate of the second specified point
     * @return the square of the distance between the two
     * sets of specified coordinates.
     * @since 1.0
     */
    public static double distanceSq(double x1, double y1,
                                    double x2, double y2)
    {
        x1 -= x2;
        y1 -= y2;
        return (x1 * x1 + y1 * y1);
    }

    /**
     * Returns the distance between two points.
     *
     * @param x1 the X coordinate of the first specified point
     * @param y1 the Y coordinate of the first specified point
     * @param x2 the X coordinate of the second specified point
     * @param y2 the Y coordinate of the second specified point
     * @return the distance between the two sets of specified
     * coordinates.
     * @since 1.0
     */
    public static double distance(double x1, double y1,
                                  double x2, double y2)
    {
        x1 -= x2;
        y1 -= y2;
        return Math.sqrt(x1 * x1 + y1 * y1);
    }

    /**
     * Returns the square of the distance from this
     * <code>Point</code> to a specified point.
     *
     * @param px the X coordinate of the specified point to be measured
     *           against this <code>Point</code>
     * @param py the Y coordinate of the specified point to be measured
     *           against this <code>Point</code>
     * @return the square of the distance between this
     * <code>Point</code> and the specified point.
     * @since 1.0
     */
    public double distanceSq(double px, double py) {
        px -= getX();
        py -= getY();
        return (px * px + py * py);
    }

    /**
     * Returns the square of the distance from this
     * <code>Point</code> to a specified <code>Point</code>.
     *
     * @param pt the specified point to be measured
     *           against this <code>Point</code>
     * @return the square of the distance between this
     * <code>Point</code> to a specified <code>Point</code>.
     * @since 1.0
     */
    public double distanceSq(Point pt) {
        double px = pt.getX() - this.getX();
        double py = pt.getY() - this.getY();
        return (px * px + py * py);
    }

    /**
     * Returns the distance from this <code>Point</code> to
     * a specified point.
     *
     * @param px the X coordinate of the specified point to be measured
     *           against this <code>Point</code>
     * @param py the Y coordinate of the specified point to be measured
     *           against this <code>Point</code>
     * @return the distance between this <code>Point</code>
     * and a specified point.
     * @since 1.0
     */
    public double distance(double px, double py) {
        px -= getX();
        py -= getY();
        return Math.sqrt(px * px + py * py);
    }

    /**
     * Returns the distance from this <code>Point</code> to a
     * specified <code>Point</code>.
     *
     * @param pt the specified point to be measured
     *           against this <code>Point</code>
     * @return the distance between this <code>Point</code> and
     * the specified <code>Point</code>.
     * @since 1.0
     */
    public double distance(Point pt) {
        double px = pt.getX() - this.getX();
        double py = pt.getY() - this.getY();
        return Math.sqrt(px * px + py * py);
    }

    /**
     * Returns a {@code String} representation of this {@code Point}. The format
     * of the {@code String} will be "(x,y)", where the quotes are not part of the
     * {@code String} and x and y are replaced by their respective underlying values.
     * 
     * @return a string representing this point
     * @since 1.0
     */
    @Override
    public String toString() {
        return "("+StringUtils.formatDouble(x)+","+StringUtils.formatDouble(y)+")";
    }
    
    /**
     * Returns whether or not this point is equal to the supplied object. Two
     * {@code Point}s are equal if and only if they are of the same class, and
     * their x and y values are equal as defined by the {@code ==} operator on doubles.
     * 
     * @param obj the point to compare
     * @return true if {@code obj} is a point with the same x and y as this point
     * @since 1.0
     */
    @Override
    public boolean equals(Object obj) {
        if (obj == null) {
            return this == null;
        }
        if (!(obj.getClass().equals(getClass()))) {
            return false;
        }
        Point pnt = (Point) obj;
        return pnt.x == x && pnt.y == y;
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public int hashCode() {
        int hash = 7;
        hash = 59 * hash + (int) (Double.doubleToLongBits(this.x)
                ^ (Double.doubleToLongBits(this.x) >>> 32));
        hash = 59 * hash + (int) (Double.doubleToLongBits(this.y)
                ^ (Double.doubleToLongBits(this.y) >>> 32));
        return hash;
    }

}
