package org.westtorrancerobotics.lib.spline.geom;

import java.util.Objects;
import org.westtorrancerobotics.lib.util.StringUtils;

/**
 * An angle value represented with an orientation and unit. Rather than force the
 * programmer to keep track of which unit is returned for each number through comments,
 * this class internally keeps track and will make conversions to and from the correct
 * units. This class also forces the programmer to consider what orientation their
 * angle is in, to avoid forgetting to convert units and thus ending up with nonsense
 * results. All classes in this library involving angles as parameters or return types
 * for public methods will not return a naked number, instead wrapping it with an
 * {@code Angle}.
 * 
 * @since 1.0
 */
public class Angle {
    
    /**
     * A copy of this angle stored at its declaration through the constructor
     * {@link #Angle(Angle)} for use in the {@code equals} comparison. In angles
     * created using the secondary duplication constructor, this value is set to
     * {@code null} to prevent infinite recursion.
     * 
     * @since 1.0
     */
    private final Angle initialMe;
    
    private double value;
    private AngleUnit unit;
    private AngleOrientation orientation;

    /**
     * Creates an angle with the specified value, unit, and orientation.
     * Ensure that the value supplied is in fact in the specified orientation and
     * unit, otherwise it will have an effectively incorrect value.
     * 
     * @param value the measured quantifier of the angle
     * @param unit the unit of the measured value
     * @param orientation the orientation of the measured value
     * @see AngleUnit
     * @see AngleOrientation
     * @since 1.0
     */
    public Angle(double value, AngleUnit unit, AngleOrientation orientation) {
        this.value = value;
        this.unit = unit;
        this.orientation = orientation;
        this.initialMe = new Angle(this);
    }
    
    public Angle(double x, double y) {
        this(Math.atan2(y, x), AngleUnit.RADIANS, AngleOrientation.UNIT_CIRCLE);
    }
    
    public Angle(Point p) {
        this(p.x, p.y);
    }
    
    public static Angle add(Angle first, Angle second, AngleOrientation orientation) {
        double firstVal = first.getValue(AngleUnit.RADIANS, orientation);
        double secondVal = second.getValue(AngleUnit.RADIANS, orientation);
        return new Angle(firstVal + secondVal, AngleUnit.RADIANS, orientation);
    }
    
    public static Angle difference(Angle minuend, Angle subtrahend, AngleOrientation orientation) {
        double firstVal = minuend.getValue(AngleUnit.RADIANS, orientation);
        double secondVal = subtrahend.getValue(AngleUnit.RADIANS, orientation);
        return new Angle(firstVal - secondVal, AngleUnit.RADIANS, orientation);
    }
    
    public Angle copy() {
        this.convert(initialMe.unit, initialMe.orientation);
        return new Angle(value, unit, orientation);
    }
    
    /**
     * Creates an angle with the same orientation, unit, and value as the original.
     * Useful because when the value of an {@code Angle} is requested, conversions
     * that change the underlying unit may occur, and so a copy of the angle can
     * be made here. Do not call the {@code getValue}, {@code getX}, {@code getY},
     * or {@code toRect} methods on an angle created with this constructor if you
     * want {@code equals} to be guaranteed, instead use the original angle. This
     * constructor is provided to allow duplication of an angle without creation of
     * a saved initial pointer, instead the private field {@code initialMe} is set
     * to null.
     * 
     * @param original the angle to replicate in orientation, unit, and value
     * @see #Angle(Angle)
     * @see #equals
     * @since 1.0
     */
    protected Angle(Angle original) {
        this.value = original.value;
        this.unit = original.unit;
        this.orientation = original.orientation;
        this.initialMe = null;
    }
    
    /**
     * Performs an underlying conversion of the {@code Angle} to the chosen unit.
     * Does nothing if the units are the same, otherwise uses the
     * {@link AngleUnit#getUnitsPerRevolution()} methods of the new and old unit to
     * convert.
     * 
     * @param newUnit the unit to which the angle should change
     * @see AngleUnit
     * @since 1.0
     */
    protected void convert(AngleUnit newUnit) {
        if (unit == newUnit) {
            return;
        }
        value *= newUnit.getUnitsPerRevolution() / unit.getUnitsPerRevolution();
        unit = newUnit;
    }
    
    /**
     * Performs an underlying conversion of the {@code Angle} to the chosen orientation.
     * Does nothing if the orientations are the same, otherwise uses arithmetic and the
     * orientation definitions of each enum constant in {@code AngleOrientation}.
     * 
     * @param newOrientation the orientation to which the angle should change
     * @see AngleOrientation
     * @since 1.0
     */
    protected void convert(AngleOrientation newOrientation) {
        if (orientation == newOrientation) {
            return;
        }
        if (newOrientation == AngleOrientation.UNSPECIFIED) {
            convert(initialMe.orientation);
            return;
        }
        if (orientation == AngleOrientation.UNSPECIFIED) {
            return;
        }
        Angle right = new Angle(90, AngleUnit.DEGREES, AngleOrientation.UNSPECIFIED);
        double minuend = right.getValue(unit, AngleOrientation.UNSPECIFIED);
        if (orientation == AngleOrientation.UNIT_CIRCLE &&
                newOrientation == AngleOrientation.COMPASS_HEADING) {
            value = minuend - value;
            orientation = newOrientation;
        }
        if (orientation == AngleOrientation.COMPASS_HEADING &&
                newOrientation == AngleOrientation.UNIT_CIRCLE) {
            value = minuend - value;
            orientation = newOrientation;
        }
    }
    
    /**
     * Performs an underlying conversion of the {@code Angle} to the chosen unit
     * and orientation. Calls first {@link #convert(AngleUnit newUnit)} and second
     * {@link #convert(AngleOrientation newOrientation)} on the underlying angle.
     * 
     * @param newUnit the unit to which the angle should change
     * @param newOrientation the orientation to which the angle should change
     * @since 1.0
     */
    protected void convert(AngleUnit newUnit, AngleOrientation newOrientation) {
        this.convert(newUnit);
        this.convert(newOrientation);
    }
    
    /**
     * Gives the raw measure of the underlying angle. Be certain that when this method
     * is called, the current values of {@code unit} and {@code orientation} are
     * known.
     * 
     * @return the raw measure of the underlying angle
     * @since 1.0
     */
    protected double getRawValue() {
        return value;
    }

    /**
     * Gives the current unit of this angle.
     * 
     * @return the current unit of this angle
     * @see AngleUnit
     * @since 1.0
     */
    protected AngleUnit getUnit() {
        return unit;
    }

    /**
     * Gives the current orientation of this angle.
     * 
     * @return the current orientation of this angle.
     * @see AngleOrientation
     * @since 1.0
     */
    protected AngleOrientation getOrientation() {
        return orientation;
    }
    
    /**
     * Converts this angle and a specified radius to rectangular coordinates.
     * The polar coordinate ({@code radius}, {@code this}) is to be converted.
     * The radius supplied will in no way be retained as a part of the angle.
     * 
     * @param radius the first polar coordinate to convert to Cartesian coordinates
     * @return a point representing the x and y components of this angle scaled by
     *         the given radius
     * @see #getX
     * @see #getY
     * @since 1.0
     */
    public Point toRect(double radius) {
        return new Point(getX() * radius, getY() * radius);
    }

    /**
     * Returns the measure of the angle in the desired units. The angle will be internally
     * converted to the supplied units and then the underlying value will be returned.
     * It is required that units are supplied to ensure the programmer is certain
     * what type of angle he or she will be mathematically manipulating in code.
     * If the angle was created in its constructor with the same units as supplied
     * here and its value has only been asked for in those same units (or has never
     * been requested), then no underlying conversions will occur.
     * 
     * @param newUnit the desired unit for the returned angle value
     * @param newOrientation the desired orientation for the returned angle value
     * @return the angle measure in the specified units and orientation
     * @see AngleUnit
     * @see AngleOrientation
     * @since 1.0
     */
    public double getValue(AngleUnit newUnit, AngleOrientation newOrientation) {
        this.convert(newUnit, newOrientation);
        return getRawValue();
    }
    
    /**
     * Gives the x value of this angle when its radius is one. Converts the angle to
     * unit circle form and takes the cosine of the underlying value. To find the
     * x coordinate of this angle with a different radius, just multiply the result
     * by that radius.
     * 
     * @return the x value of this angle
     * @see #toRect
     * @since 1.0
     */
    public double getX() {
        return Math.cos(getValue(AngleUnit.RADIANS, AngleOrientation.UNIT_CIRCLE));
    }
    
    /**
     * Gives the y value of this angle when its radius is one. Converts the angle to
     * unit circle form and takes the sine of the underlying value. To find the
     * y coordinate of this angle with a different radius, just multiply the result
     * by that radius.
     * 
     * @return the y value of this angle
     * @see #toRect
     * @since 1.0
     */
    public double getY() {
        return Math.sin(getValue(AngleUnit.RADIANS, AngleOrientation.UNIT_CIRCLE));
    }

    /**
     * Returns true if and only if {@code obj} is another angle created with the same
     * value, unit, and orientation. Two angles created by the constructor
     * {@link #Angle(double, AngleUnit, AngleOrientation)} are guaranteed to be considered
     * equal if and only if the same parameters were used in their constructor,
     * while two angles created with the {@link #Angle(Angle)} constructor will be
     * considered equal if they have equivalent values when converted to the same
     * units, and if the {@link #getValue} method on each angle has been most recently
     * called with the same parameters on each (or if it has never been called, the
     * parameters in the constructor will be referenced).
     * <p>
     * Phrased differently, two {@code Angle} objects are equal if and only if
     * they have the same value and the same initial units. In an angle created with
     * the primary constructor, value and initial units are as specified in the constructor.
     * For angles created with the duplication constructor, the value is the value
     * assigned to the single parameter. The initial units begin as those of the
     * single parameter, but whenever {@code getValue} is called, the initial units
     * will be reset to the parameters of the {@code getValue} method.
     * 
     * @param obj the angle to compare
     * @return true if {@code obj} is an angle with the same value, unit, and orientation
     *         as this angle
     * @see #Angle(Angle)
     * @see #Angle(double, AngleUnit, AngleOrientation)
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
        Angle ang = (Angle) obj;
        if (initialMe == null) {
            if (ang.initialMe == null) {
                return ang.orientation == orientation && ang.unit == unit && ang.value == value;
            } else {
                return equals(ang.initialMe);
            }
        } else {
            return initialMe.equals(ang);
        }
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public int hashCode() {
        int hash = 3;
        hash = 37 * hash + (int) (Double.doubleToLongBits(this.value) ^ (Double.doubleToLongBits(this.value) >>> 32));
        hash = 37 * hash + Objects.hashCode(this.unit);
        hash = 37 * hash + Objects.hashCode(this.orientation);
        return hash;
    }

    /**
     * Gives a string representation of the angle magnitude. Orientation is not
     * included in the result of this method. The {@code String} returned will be
     * the concatenation of the value of the {@code Angle} and of the first three
     * letters of its unit, with a delimiting space.
     * 
     * @return a string representation of the angle magnitude
     * @since 1.0
     */
    @Override
    public String toString() {
        String unitStr = unit.name().substring(0, 3);
        return StringUtils.formatDouble(value) + " " + unitStr;
    }
    
    /**
     * Enum type that indicates the magnitude of an angle. The angle unit affects
     * the number assigned to a value of a single revolution: the chosen angle unit's
     * value for {@link #getUnitsPerRevolution()}. For example, with
     * radians, this number is 2*pi, and with degrees this number is 360.
     * 
     * @since 1.0
     */
    public enum AngleUnit {
        
        /**
         * Two time pi units per revolution. Defined as the quotient of the arc
         * length of a portion of a circle and its radius.
         * 
         * @since 1.0
         */
        RADIANS(2*Math.PI),
        
        /**
         * Four hundred units per revolution. This makes each quadrant or right
         * angle 100 units, allowing ease of identification of quadrant breaks and
         * effective halving of quadrants.
         * 
         * @since 1.0
         */
        GRADIANS(400),
        
        /**
         * Three hundred sixty units per revolution. Based on the ancient Mesopotamian
         * convention of use of base sixty, this unit allows for ease of division
         * of a revolution into many fractions.
         * 
         * @since 1.0
         */
        DEGREES(360),
        
        /**
         * Twenty-one thousand six hundred units per revolution. A sixtieth of a degree.
         * 
         * @see #DEGREES
         * @since 1.0
         */
        MINUTES(21600),
        
        /**
         * One million two hundred ninety-six thousand units per revolution. A
         * sixtieth of a minute, or a thirty-six hundredth of a degree.
         * 
         * @see #DEGREES
         * @see #MINUTES
         * @since 1.0
         */
        SECONDS(1296000),
        
        /**
         * One unit per revolution.
         * 
         * @since 1.0
         */
        REVOLUTIONS(1);

        private final double unitsPerRevolution;
        
        AngleUnit(double unitsPerRevolution) {
            this.unitsPerRevolution = unitsPerRevolution;
        }

        /**
         * Returns the number assigned to a single revolution in this angle unit.
         * For example, with radians, this number is 2*pi, and with degrees this
         * number is 360.
         * 
         * @return the number of units in one revolution
         * @since 1.0
         */
        public double getUnitsPerRevolution() {
            return unitsPerRevolution;
        }
        
    }
    
    /**
     * Enum type that indicates the orientation of the zero angle and the direction
     * of increasing angle measure.
     * 
     * @since 1.0
     */
    public enum AngleOrientation {
        
        /**
         * Constant representing an angle oriented like a compass. The zero direction
         * is positive on the y axis and without magnitude on the x axis (up), and the
         * angle measure increases clockwise.
         * 
         * @since 1.0
         */
        COMPASS_HEADING,
        
        /**
         * Constant representing an angle oriented like a unit circle. The zero direction
         * is positive on the x axis and without magnitude on the y axis (right), and the
         * angle measure increases counterclockwise.
         * 
         * @since 1.0
         */
        UNIT_CIRCLE,
        
        /**
         * Indicates an angle with magnitude but not specific or important orientation.
         * <ul>
         * <li>If an angle with this orientation is asked for its value in this orientation,
         * the value specified in the constructor will return, as is expected.
         * <li>If an angle with this orientation is asked for its value in another orientation,
         * no conversion of the underlying angle will occur, and the value specified
         * in the constructor will be returned.
         * <li>If an angle with another orientation is asked for its value in this orientation,
         * the underlying angle will be converted to its original orientation, and
         * the value specified in the constructor will be returned.
         * </ul>
         * 
         * @since 1.0
         */
        UNSPECIFIED
    }
    
    public static final Angle NORTH      = new Angle(0,   AngleUnit.DEGREES, AngleOrientation.COMPASS_HEADING);
    public static final Angle NORTH_EAST = new Angle(45,  AngleUnit.DEGREES, AngleOrientation.COMPASS_HEADING);
    public static final Angle EAST       = new Angle(90,  AngleUnit.DEGREES, AngleOrientation.COMPASS_HEADING);
    public static final Angle SOUTH_EAST = new Angle(135, AngleUnit.DEGREES, AngleOrientation.COMPASS_HEADING);
    public static final Angle SOUTH      = new Angle(180, AngleUnit.DEGREES, AngleOrientation.COMPASS_HEADING);
    public static final Angle SOUTH_WEST = new Angle(225, AngleUnit.DEGREES, AngleOrientation.COMPASS_HEADING);
    public static final Angle WEST       = new Angle(270, AngleUnit.DEGREES, AngleOrientation.COMPASS_HEADING);
    public static final Angle NORTH_WEST = new Angle(315, AngleUnit.DEGREES, AngleOrientation.COMPASS_HEADING);
    
}
