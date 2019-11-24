package org.westtorrancerobotics.lib.functionmath.casters;

import java.util.Objects;
import org.westtorrancerobotics.lib.functionmath.interfaces.CalculusFunction;
import org.westtorrancerobotics.lib.functionmath.interfaces.DerivableFunction;
import org.westtorrancerobotics.lib.functionmath.interfaces.DerivableIntegrableFunction;
import org.westtorrancerobotics.lib.functionmath.interfaces.Function;
import org.westtorrancerobotics.lib.functionmath.interfaces.IntegrableFunction;

/**
 * Makes a function that is an instance of {@code DerivableFunction} into an instance
 * of {@code CalculusFunction}. Uses the class {@link BruteIntegral} to gain its
 * additional functionality.
 * 
 * @since 1.0
 */
public class Calculify implements CalculusFunction {
    
    private final DerivableFunction input;
    private final int INTERGAL_NUM_SAMPLES;
    private final double MAX_INPUT;
    private final DerivableFunction integral;

    /**
     * Boxes the function {@code input}, leaving it with the same output and input
     * characteristics. The function gains the ability to be used as a
     * {@code CalculusFunction}, through use of estimation done in the class
     * {@link BruteIntegral}.
     * 
     * @param input the function to box
     * @param INTERGAL_NUM_SAMPLES the number of cubic pieces to be used when and if
     *                             this function is integrated
     * @param MAX_INPUT the maximum input to be used when and if this function is
     *                  integrated
     * @see BruteIntegral#BruteIntegral(DerivableFunction, int, double) 
     * @since 1.0
     */
    public Calculify(DerivableFunction input, int INTERGAL_NUM_SAMPLES, double MAX_INPUT) {
        this.input = input;
        this.INTERGAL_NUM_SAMPLES = INTERGAL_NUM_SAMPLES;
        this.MAX_INPUT = MAX_INPUT;
        integral = new BruteIntegral(input, INTERGAL_NUM_SAMPLES, MAX_INPUT);
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public CalculusFunction derivative() {
        DerivableFunction deriv = input.derivative();
        return new Calculify(
                new DerivableIntegrableFunction<DerivableFunction>() {
                    @Override
                    public double get(double x) {
                        return deriv.get(x);
                    }
                    @Override
                    public DerivableFunction integral() {
                        return input;
                    }
                    @Override
                    public DerivableIntegrableFunction<?> derivative() {
                        return new Calculify(deriv.derivative(), INTERGAL_NUM_SAMPLES, MAX_INPUT);
                    }
                    @Override
                    public String toString() {
                        return deriv.toString();
                    }
                },
                INTERGAL_NUM_SAMPLES, MAX_INPUT
        );
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public CalculusFunction integral() {
        if (input instanceof IntegrableFunction) {
            Function integ = ((IntegrableFunction) input).integral();
            if (integ instanceof CalculusFunction) {
                return (CalculusFunction) integ;
            } else if (integ instanceof DerivableFunction) {
                return new Calculify((DerivableFunction) integ, INTERGAL_NUM_SAMPLES, MAX_INPUT);
            }
        }
        return new Calculify(integral, INTERGAL_NUM_SAMPLES, MAX_INPUT);
    }

    /**
     * Returns the output of the function supplied in initialization.
     * 
     * @param x the input for the original function
     * @return the output of the original function
     * @since 1.0
     */
    @Override
    public double get(double x) {
        return input.get(x);
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public String toString() {
        return input.toString();
    }
    
    /**
     * 
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
        Calculify cfy = (Calculify) obj;
        return cfy.input.equals(input) && cfy.INTERGAL_NUM_SAMPLES == INTERGAL_NUM_SAMPLES && cfy.MAX_INPUT == MAX_INPUT;
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public int hashCode() {
        int hash = 3;
        hash = 59 * hash + Objects.hashCode(this.input);
        hash = 59 * hash + this.INTERGAL_NUM_SAMPLES;
        hash = 59 * hash + (int) (Double.doubleToLongBits(this.MAX_INPUT) ^ (Double.doubleToLongBits(this.MAX_INPUT) >>> 32));
        return hash;
    }
    
}