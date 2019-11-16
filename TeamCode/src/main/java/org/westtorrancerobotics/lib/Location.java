package org.westtorrancerobotics.lib;

import java.util.Objects;

/**
 * The {@code Location} class defines a point representing a location including direction
 * in {@code (x,y)} coordinate space with double precision. It contains an x and
 * a y coordinate, and methods to manipulate those coordinates, as well as an
 * {@link Angle}. Can be used as though it has no location as a {@code Point}.
 * 
 * @see Point
 * @since 1.0
 */
public class Location extends Point {

    public static Location ORIGIN = new Location(0, 0, Angle.NORTH);

    /**
     * The direction of this {@code Location}, as an {@link Angle}.
     * 
     * @since 1.0
     */
    public Angle direction;

    /**
     * Creates a new {@code Location} with the specified coordinates and direction.
     * 
     * @param x the x coordinate of the {@code Location}, fed to the super {@code Point} object 
     * @param y the y coordinate of the {@code Location}, fed to the super {@code Point} object 
     * @param direction the angle of the {@code Location}
     * @see Point#Point(double, double)
     * @since 1.0
     */
    public Location (double x, double y, Angle direction) {
        super(x,y);
        this.direction = direction;
    }
    
    public Location setOrigin(Location center) {
        double x2 = this.x;
        double y2 = this.y;
        Angle direction2 = this.direction;
        x2 -= center.x;
        y2 -= center.y;
        double r = Math.hypot(x2, y2);
        double theta0 = Math.atan2(y2, x2);
        double theta1 = center.direction.getValue(Angle.AngleUnit.RADIANS, Angle.AngleOrientation.UNIT_CIRCLE);
        Point p = new Point(r, new Angle(theta0 + theta1, Angle.AngleUnit.RADIANS, Angle.AngleOrientation.UNIT_CIRCLE));
        direction2 = Angle.difference(direction2, center.direction, Angle.AngleOrientation.COMPASS_HEADING);
        return new Location(p.x, p.y, direction2);
    }

    /**
     * Returns a {@code String} representation of the {@code Location}. The {@code String}
     * will be of the format "[angle] @ (x,y)", with no quotation marks in the output,
     * and with [angle] replaced by the result of the {@link Angle#toString()} method
     * of the direction of this {@code Location}, and with x and y replaced by the
     * actual underlying numerical values.
     * 
     * @return a string representation of the location
     * @since 1.0
     */
    @Override
    public String toString() {
        return direction.toString() + " @ " + super.toString();
    }
    
    /**
     * Returns whether or not this location is equal to the supplied object. Two
     * {@code Location}s are equal if and only if they are of the same class,
     * their x and y values are equal as defined by the {@code ==} operator on doubles,
     * and the directions are equal.
     * 
     * @param obj the location to compare
     * @return true if {@code obj} is an point with the same x, y, and direction as this point
     * @see Angle#equals
     * @since 1.0
     */
    @Override
    public boolean equals(Object obj) {
        if (obj == null) {
            return false;
        }
        if (!(obj.getClass().equals(getClass()))) {
            return false;
        }
        if (!super.equals(obj)) {
            return false;
        }
        Location loc = (Location) obj;
        return loc.direction.equals(direction);
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public int hashCode() {
        int hash = 5;
        hash = 67 * hash + Objects.hashCode(this.direction);
        return hash;
    }
}
