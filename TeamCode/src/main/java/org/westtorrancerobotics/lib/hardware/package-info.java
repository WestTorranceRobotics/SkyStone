/**
 * Replaces FIRST library dependencies with interfaces that can be made to contain
 * references to the FIRST provided code. Rather than to include large amounts of
 * code that would allow direct use of hardware and standardized control algorithms,
 * the ability to implement this hardware and control on the user end allows compatability
 * with various robotics programs and contexts, as well as persistence through external
 * software modifications. The open ended implementation of standard control algorithms
 * allows not only for use of pre-made functions and objects but also for the modification
 * of those functions and objects, and for the creation of new, custom functions and
 * objects with the purpose of more precise tailoring to the needs of a specific robot.
 * 
 * @since 1.0
 */
package org.westtorrancerobotics.lib.hardware;