package org.westtorrancerobotics.lib.functionmath.interfaces;

import org.westtorrancerobotics.lib.functionmath.Polynomial;

/**
 * Interface for a mathematical function taking a double input and giving a double
 * output. Functions like {@code y = 2x + 1} can be implemented here. For functions
 * with derivatives or integrals that can be computed, or with known inverses, other
 * interfaces in this package can be overridden to thus indicate.
 * 
 * @see org.westtorrancerobotics.lib.functionmath.interfaces
 * @since 1.0
 */
@FunctionalInterface
public interface Function extends java.util.function.DoubleUnaryOperator {
    
    /**
     * Gives the output of this function at the given input. The x (domain) value
     * should be given as a floating-point number with decimal precision, and the
     * same type of input will be returned for the y (range) value.
     * 
     * @param x the input on which to execute to function
     * @return the output for the function at the specified input
     * @since 1.0
     */
    public double get(double x);

    /**
     * Gives a LaTeX representation of this function. LaTeX functions can be rendered
     * using numerous Internet tools, and can even be graphed using some tools. The
     * intent is that the string returned will give the best balance between readability
     * like that of a LaTeX code created by a human while representing the internal
     * processing technique of the function.
     * 
     * @return a LaTeX representation of this function
     */
    @Override
    public String toString();

    /**
     * Tells whether or not the function is the same as the specified object. Equality
     * requires that both objects are the same class, and that their outputs will
     * be equal at any given input. The output comparison is done through a checking
     * of all output-affecting parameters specified in the constructor, rather than
     * through verification of individual outputs.
     * 
     * @param obj the object to compare
     * @return true if the two objects are equal
     */
    @Override
    public boolean equals(Object obj);
    
    /**
     * {@inheritDoc}
     * @see #get(double x)
     * @since 1.0
     */
    @Override
    public default double applyAsDouble(double operand) {
        return get(operand);
    }
    
    /**
     * Returns a {@link ComplexFunction} that returns the input. Expressed in typical
     * mathematical syntax, this could be {@code y = x} or {@code f(n) = n}.
     * 
     * @return the identity function
     * @since 1.0
     */
    public static ComplexFunction identity() {
        return new ComplexFunction () {
            @Override
            public CalculusFunction derivative() {
                return new Polynomial(1, 0).derivative();
            }
            @Override
            public CalculusFunction integral() {
                return new Polynomial(1, 0).integral();
            }
            @Override
            public double get(double x) {
                return x;
            }
            @Override
            public InversibleFunction inverse() {
                return this;
            }
            @Override
            public boolean equals(Object obj) {
                if (obj == null) {
                    return this == null;
                }
                return obj.getClass().equals(getClass());
            }
            @Override
            public int hashCode() {
                return getClass().hashCode() + 1;
            }
            @Override
            public String toString() {
                return "x";
            }
        };
    }
    
    /**
     * Regular expression that matches all instances of the string {@code "x"} that
     * are not followed by a LaTeX subscript. Subscripts are used within functions
     * to names new variables that are integration variables of some portion of the
     * function, and these integration variables should be preserved when all instances
     * of the input variable string {@code "x"} are being replaced with some other
     * expression, whether the string representation of another function or a new
     * variable. The following code template creates a {@code String} replacing all
     * copies of the input variable string in the existing {@code String outStr}
     * with the existing {@code String inStr}:
     * <code>outStr.replaceAll(Function.xVariableRegex(), Matcher.quoteReplacement(inStr))</code>
     * It can be used as a model for string replacements that use this regex, as
     * is its intended use.
     * 
     * @return a regex representing the input variable of a function's LaTeX string
     * @see #toString()
     * @since 1.0
     */
    public static String xVariableRegex() {
        return "x(?!_\\{\\d+\\})";
    }
}
