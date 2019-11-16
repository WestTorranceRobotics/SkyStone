package org.westtorrancerobotics.lib;

import java.math.BigDecimal;

/**
 * Utility class containing methods for reformatting and performing functions on
 * {@code String}s.
 * 
 * @since 1.0
 */
public class StringUtils {
    
    private StringUtils() {} // no constructor
    
    /**
     * Formats a double into the numerical format in which it would usually be written
     * by a human. Uses a maximum of 6 decimal places.
     * 
     * @param number the double to format
     * @return a readable string representing the double
     * @throws NumberFormatException if the input is {@code NaN}
     * @see #formatDouble(double, int)
     * @since 1.0
     */
    public static String formatDouble(double number) {
        return formatDouble(number, 6);
    }
    
    /**
     * Formats a double into the numerical format in which it would usually be written
     * by a human. Uses the maximum number of decimal places specified. If scientific
     * notation is the default display method for the function, the LaTeX formatting
     * (which includes a carat and then a power included in curly braces) is used.
     * 
     * @param number the double to format
     * @param numDecimals the number of decimal places in the formatted decimal
     * @return a readable string representing the double
     * @throws NumberFormatException if the input is {@code NaN}
     * @see #formatDouble(double, int)
     * @since 1.0
     */
    public static String formatDouble(double number, int numDecimals) {
        if (!Double.isFinite(number)) {
            return String.valueOf(number);
        }
        String coString = String.valueOf(number);
        double valueToRound = number;
        double strt1 = System.nanoTime();
        if (coString.contains("E")) {
            String[] coStrings = coString.split("E");
            valueToRound = Double.valueOf(coStrings[0]);
            coString = "\\cdot 10^{" + coStrings[1] + "}";
        } else {
            coString = "";
        }
        String str = BigDecimal.valueOf(valueToRound).setScale(numDecimals, BigDecimal.ROUND_HALF_UP).toPlainString();
        str = str.replaceFirst("0+(?!.)\\b", "");
        if (str.endsWith(".")) {
            str = str.substring(0, str.length() - 1);
        }
        return str + coString;
    }
    
    /**
     * Takes the string given and encrypts it lossy converts it into a long. Useful
     * for providing a class a specific, likely unique number based on its name,
     * in a way that will not change each runtime.
     * 
     * @param plain the string to encode
     * @return a {@code long} representation of the string
     */
    public static long encodeAsLong(String plain) {
        long code = 17;
        for (char c : plain.toCharArray()) {
            code = code * 29 + c;
        }
        return code;
    }
    
}
