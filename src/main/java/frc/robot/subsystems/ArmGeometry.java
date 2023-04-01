package frc.robot.subsystems;

import org.a05annex.util.AngleD;
import org.a05annex.util.Utl;
import org.jetbrains.annotations.NotNull;

import java.awt.geom.Point2D;

/**
 * This is a helper class that contains all the geometry math for the arm subsystem. The math is
 * abstracted into this class so we can easily do automated testing, and so arm geometry information
 * is available to other subsystems and commands that may need access to that information.
 */
public class ArmGeometry {

    /**
     * A class representing arm positions
     */
    public static class ArmPosition {
        private double pivotPosition;
        private double extensionPosition;

        ArmPosition(double pivotPosition, double extensionPosition) {
            this.pivotPosition = pivotPosition;
            this.extensionPosition = extensionPosition;
        }

        public double getPivotPosition() {
            return pivotPosition;
        }

        public double getExtensionPosition() {
            return extensionPosition;
        }

        /**
         * @return {@code true} if the current values were valid for the robot during setup,
         * {@code false} if the values were clipped in some way to make the position valid at setup.
         */
        public boolean clipToValidAtSetup() {
            return true;
        }

        /**
         * Clip the position so all positions are valid and so that the end of the arm is inside the
         * allowable extension zone (48" horizontally beyond the frame, and maximum height of 78"
         * above the ground. The strategy is to
         * first clip the positions to the allowable bounds, then that is converted to location, and if the
         * location is outside the extension zone or below the ground, the arm is shortened to make the position
         * without changing the pivot angle.
         *
         * @return {@code true} if the current values were valid for competition play,
         * {@code false} if the values were clipped in some way to make the position valid during play.
         */
        public boolean clipToValidInPlay() {
            double originalPivotPosition = pivotPosition;
            double originalExtensionPosition = extensionPosition;
            // first clip - clip the positions to the valid position ranges
            pivotPosition = Utl.clip(pivotPosition, PIVOT_MIN_POSITION, PIVOT_MAX_POSITION);
            extensionPosition = (EXTENSION_POSITION_MIN_ALLOWABLE < EXTENSION_POSITION_MAX_ALLOWABLE) ?
                    Utl.clip(extensionPosition, EXTENSION_POSITION_MIN_ALLOWABLE,EXTENSION_POSITION_MAX_ALLOWABLE) :
                    Utl.clip(extensionPosition, EXTENSION_POSITION_MAX_ALLOWABLE,EXTENSION_POSITION_MIN_ALLOWABLE);
            // get back to a position
            Point2D.Double location = getArmLocationFromPositions(pivotPosition,extensionPosition);
            AngleD pivotAngle = getPivotAngleFromPosition(pivotPosition);
            boolean clippedLocation = false;
            // check position against the extension zone boundary
            double maxX = HORIZONTAL_ARM_PIVOT_TO_FRAME + HORIZONTAL_EXTENSION_PAST_FRAME_LIMIT -
                    HORIZONTAL_EXTENSION_SAFETY_BUFFER;
            if (location.x > maxX) {
                location.x = maxX;
                location.y = maxX / pivotAngle.tan();
                clippedLocation = true;
            }
            // check the position against the ground boundary
            double minY = (-VERTICAL_FLOOR_TO_ARM_PIVOT) + FLOOR_EXTENSION_SAFETY_BUFFER;
            if (location.y < minY) {
                location.x = minY * pivotAngle.tan();
                location.y = minY;
                clippedLocation = true;
            }
            // check the position against the height boundary
            double maxY = HEIGHT_LIMIT - HORIZONTAL_EXTENSION_SAFETY_BUFFER;
            if (location.y > maxY) {
                location.y = maxY;
                clippedLocation = true;
            }

            // OK, now if the location was clipped, get the new positions
            if (clippedLocation) {
                ArmPosition newPositions = getArmPositionsFromLocation(location.x, location.y);
                pivotPosition = newPositions.pivotPosition;
                extensionPosition = newPositions.extensionPosition;
            }

            return (originalPivotPosition == pivotPosition) && (originalExtensionPosition == extensionPosition);
        }
    }

    public static final double PIVOT_TICS_PER_ROTATION = 30.309 * 4.0; //Reading from 0 to 90 degrees * 4 = full rotation

    public static final double PIVOT_MIN_POSITION = -40.0;
    public static final double PIVOT_MAX_POSITION = 45.0;

    /**
     * The extension position encoder when the arm is against the minimum extension limit.
     */
    public static final double EXTENSION_POSITION_MIN_EXTENSION = 0.0;
    /**
     * The arm length (in inches) physically measured when the arm is against the minimum extension limit.
     */
    public static final double ARM_LENGTH_MIN_EXTENSION = 33.375;
    /**
     * The extension position encoder when the arm is against the maximum extension limit.
     */
    public static final double EXTENSION_POSITION_MAX_EXTENSION = 150.73;
    /**
     * The arm length (in inches) physically measured when the arm is against the maximum extension limit.
     */
    public static final double ARM_LENGTH_MAX_EXTENSION = 72.0;
    /**
     * This is a cushion in the requested extension position that allows about 3/8" between the minimum-maximum
     * end position and the physical stop. This buffer insures that the requested encoder position is inside
     * the stop so the extension motor is never stalled trying to exceed the stop.
     */
    public static final double EXTENSION_POSITION_CUSHION = 0.0;

    @SuppressWarnings("ConstantValue")
    public static final double EXTENSION_POSITION_MIN_ALLOWABLE = EXTENSION_POSITION_MIN_EXTENSION +
            (EXTENSION_POSITION_MIN_EXTENSION > EXTENSION_POSITION_MAX_EXTENSION ?
                    -EXTENSION_POSITION_CUSHION : EXTENSION_POSITION_CUSHION);
    @SuppressWarnings("ConstantValue")
    public static final double EXTENSION_POSITION_MAX_ALLOWABLE = EXTENSION_POSITION_MAX_EXTENSION +
            (EXTENSION_POSITION_MIN_EXTENSION > EXTENSION_POSITION_MAX_EXTENSION ?
                    EXTENSION_POSITION_CUSHION : -EXTENSION_POSITION_CUSHION);
    /**
     * The extension encoder tics per inch, calculated from the tics between
     */
    public static final double EXTENSION_TICS_PER_INCH =
            (EXTENSION_POSITION_MAX_EXTENSION - EXTENSION_POSITION_MIN_EXTENSION) /
                    (ARM_LENGTH_MAX_EXTENSION - ARM_LENGTH_MIN_EXTENSION);

    /**
     * The horizontal distance (in inches) from the arm pivot to the edge of the frame.
     */
    public static final double HORIZONTAL_ARM_PIVOT_TO_FRAME = 13.5;
    /**
     * The vertical distance from the floor to the arm pivot.
     */
    public static final double VERTICAL_FLOOR_TO_ARM_PIVOT = 20.25; // Need a measurement

    /**
     * The horizontal distance (in inches) the game rules allow a robot appendage to extend from the frame.
     */
    public static final double HORIZONTAL_EXTENSION_PAST_FRAME_LIMIT = 48.0;

    public static final double HEIGHT_LIMIT = 78.0;
    /**
     * The horizontal distance safety buffer (in inches) to protect from inertial effects that may prevent
     * the arm control from explicitly following the specified settings.
     */
    public static final double HORIZONTAL_EXTENSION_SAFETY_BUFFER = 1.0;
    public static final double FLOOR_EXTENSION_SAFETY_BUFFER = 8.0;

    /**
     * The method computes the current arm length from the reported extension encoder position.
     *
     * @return The current arm length
     */
    public static double getArmLengthFromExtensionPosition(double extensionPosition) {
        return ARM_LENGTH_MIN_EXTENSION + (((extensionPosition - EXTENSION_POSITION_MIN_EXTENSION) /
                (EXTENSION_POSITION_MAX_EXTENSION - EXTENSION_POSITION_MIN_EXTENSION)) *
                (ARM_LENGTH_MAX_EXTENSION - ARM_LENGTH_MIN_EXTENSION));
    }

    /**
     * This method computes the extension encoder position that would produce the requested arm length if the
     * geometry of the arm were unconstrained. It is the caller's responsibility to determine what should be
     * done if the encoder position is outside the physical limits of the arm.
     *
     * @param armLength The desired arm length.
     * @return The encoder position to achieve the requested length.
     */
    public static double getExtensionPositionFromArmLength(double armLength) {

        return EXTENSION_POSITION_MIN_EXTENSION +
                (((armLength - ARM_LENGTH_MIN_EXTENSION) /
                        (ARM_LENGTH_MAX_EXTENSION - ARM_LENGTH_MIN_EXTENSION)) *
                        (EXTENSION_POSITION_MAX_EXTENSION - EXTENSION_POSITION_MIN_EXTENSION));
    }

    /**
     * Get the pivot angle from the pivot position. This transformation does no
     * validity checking or clipping.
     *
     * @param pivotPosition The pivot position.
     * @return The pivot angle corresponding to the pivot position.
     */
    public static AngleD getPivotAngleFromPosition(double pivotPosition) {
        return new AngleD().setDegrees((pivotPosition * 360.0) / PIVOT_TICS_PER_ROTATION);
    }

    /**
     * Get the pivot position from the pivot angle. This transformation does no
     * validity checking or clipping.
     *
     * @param pivotAngle The pivot angle.
     * @return The pivot position corresponding to the pivot angle.
     */
    public static double getPivotPositionFromAngle(AngleD pivotAngle) {
        return (pivotAngle.getDegrees() * PIVOT_TICS_PER_ROTATION) / 360.0;

    }

    /**
     * Convert pivot and extension positions to a location in 2D space where the pivot
     * is at (0.0,0.0), X is horizontal towards the frame, and Y is vertical. This
     * method does not consider validity of the location or the encoder positions when making
     * this conversion.
     *
     * @param pivotPosition The pivot position.
     * @param extensionPosition The extension position.
     * @return The location of the end of the arm.
     */
    @NotNull
    public static Point2D.Double getArmLocationFromPositions(double pivotPosition, double extensionPosition) {
        AngleD pivotAngle = getPivotAngleFromPosition(pivotPosition);
        double armLength = getArmLengthFromExtensionPosition(extensionPosition);
        return new Point2D.Double(armLength * pivotAngle.sin(), armLength * pivotAngle.cos());
    }

    /**
     * Convert a position relative to the pivot at (0.0,0.0) to encoder positions to achieve that position. This
     * method does not consider validity of the location or the encoder positions when making this conversion.
     *
     * @param x The X position relative to the pivot at (0.0,0.0).
     * @param y The Y position relative to the pivot at (0.0,0.0).
     * @return Returns the encoder positions required to position the arm at the specified location.
     */
    @NotNull
    public static ArmGeometry.ArmPosition getArmPositionsFromLocation(double x, double y) {
        double armLength = Utl.length(x, y);
        AngleD pivotAngle = new AngleD().atan2(x, y);
        return new ArmPosition(getPivotPositionFromAngle(pivotAngle), getExtensionPositionFromArmLength(armLength));
    }
}
