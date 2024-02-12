package com.swervedrivespecialties.swervelib;

@FunctionalInterface
public interface AbsoluteEncoder {
    /**
     * Gets the current angle reading of the encoder in radians.
     *
     * @return The current angle in radians. Range: [0, 2pi)
     */
    double getAbsoluteAngle();

    /**
     * Are we ok to use encoder Absolute Angle?
     */
    default boolean isEncoderOK() {
        return false;
    };

    /**
     * Returns the internal encoder object, if applicable
     * 
     * @return The internal encoder object.
     */
    default Object getInternal() {
        return null;
    }
}
