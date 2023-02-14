package frc.robot.subsystems;

import org.a05annex.util.AngleD;
import org.a05annex.util.Utl;

import java.awt.geom.Point2D;

/**
 * This is a helper class that contains all of the math for the arm subsystem. The math is
 * abstracted into this class so we can easily do automated testing, and so arm geometry information
 * is available to other subsystems and commands that may need access to that information.
 */
public class ArmGeometry {

    public static class ArmPositions {
        public final double pivotPosition;
        public final double extensionPosition;
        ArmPositions(double pivotPosition, double extensionPosition) {
            this.pivotPosition = pivotPosition;
            this.extensionPosition = extensionPosition;
        }
    }

    public static final double PIVOT_TICS_PER_ROTATION = 30.309 * 4.0; //Reading from 0 to 90 degrees * 4 = full rotation

    public static final double PIVOT_MIN_POSITION = 0.0;
    public static final double PIVOT_MAX_POSITION = 45.0;

    /**
     * The extension position encoder when the arm is against the minimum extension limit.
     */
    public static final double EXTENSION_POSITION_MINIMUM_EXTENSION = 210;
    /**
     * The arm length (in inches) physically measured when the arm is against the minimum extension limit.
     */
    public static final double ARM_LENGTH_MINIMUM_EXTENSION = 22.5;
    /**
     * The extension position encoder when the arm is against the maximum extension limit.
     */
    public static final double EXTENSION_POSITION_MAXIMUM_EXTENSION = 0;
    /**
     * The arm length (in inches) physically measured when the arm is against the maximum extension limit.
     */
    public static final double ARM_LENGTH_MAXIMUM_EXTENSION = 77.78;
    /**
     * This is a cushion in the requested extension position that allows about 3/8" between the menimum-maximum
     * end position and the physical stop. This buffer insures that the requested encoder position is inside
     * the stop so the extension motor is never stalled trying to exceed the stop.
     */
    public static final double EXTENSION_POSITION_CUSHION = 1.5;
    /**
     * The extension encoder tics per inch, calculated from the tics between
     */
    public static final double EXTENSION_TICS_PER_INCH =
            (EXTENSION_POSITION_MAXIMUM_EXTENSION - EXTENSION_POSITION_MINIMUM_EXTENSION) /
                    (ARM_LENGTH_MAXIMUM_EXTENSION - ARM_LENGTH_MINIMUM_EXTENSION);

    /**
     * The horizontal distance (in inches) from the arm pivot to the edge of the frame.
     */
    public static final double HORIZONTAL_ARM_PIVOT_TO_FRAME = 14.0;
    /**
     * The vertical distance fr4om the floor to the arm pivot.
     */
    public static final double VERTICAL_FLOOR_TO_ARM_PIVOT = 18.0; // Need a real distance for this, my guess.

    /**
     * The horizontal distance (in inches) the game rules allow a robot appendage to extend from the frame.
     */
    public static final double HORIZONTAL_EXTENSION_PAST_FRAME_LIMIT = 48.0;

    /**
     * The horizontal distance safety buffer (in inches) to protect from inertial effects that may prevent
     * the arm control from explicitly following the specified settings.
     */
    public static final double HORIZONTAL_EXTENSION_SAFETY_BUFFER = 5.0;

    /**
     * The method computes the current arm length from the reported extension encoder position.
     * @return The current arm length
     */
    public static double getArmLengthFromExtensionPosition(double extensionPosition) {
        return ARM_LENGTH_MINIMUM_EXTENSION + (((extensionPosition - EXTENSION_POSITION_MINIMUM_EXTENSION)/
                (EXTENSION_POSITION_MAXIMUM_EXTENSION - EXTENSION_POSITION_MINIMUM_EXTENSION)) *
                (ARM_LENGTH_MAXIMUM_EXTENSION - ARM_LENGTH_MINIMUM_EXTENSION));
    }

    /**
     * This method computes the extension encoder position that would produce the requested arm length if the
     * geometry of the arm were unconstrained. It is the caller's responsibility to determine what should be
     * done if the encoder position is outside the physical limits of the arm.
     * @param armLength The desired arm length.
     * @return The encoder position to achieve the requested length.
     */
    public static double getExtensionPositionFromArmLength(double armLength) {

        return  EXTENSION_POSITION_MINIMUM_EXTENSION +
                (((armLength - ARM_LENGTH_MINIMUM_EXTENSION) /
                        (ARM_LENGTH_MAXIMUM_EXTENSION - ARM_LENGTH_MINIMUM_EXTENSION)) *
                        (EXTENSION_POSITION_MAXIMUM_EXTENSION - EXTENSION_POSITION_MINIMUM_EXTENSION));
    }

    public static AngleD getPivotAngleFromPosition(double pivotPosition) {
        return new AngleD().setDegrees((pivotPosition * 360.0)/ PIVOT_TICS_PER_ROTATION);

    }
    public static double getPivotPositionFromAngle(AngleD pivotAngle) {
        return (pivotAngle.getDegrees() * PIVOT_TICS_PER_ROTATION) / 360.0;

    }
    public static Point2D.Double getArmLocationFromPositions(double pivotPosition, double extensionPosition) {
        return new Point2D.Double(0.0, 0.0);
    }

    public static ArmPositions getArmPositionsFromLocation(double x, double y) {
        return new ArmPositions(0.0, EXTENSION_POSITION_MAXIMUM_EXTENSION);
    }
}
