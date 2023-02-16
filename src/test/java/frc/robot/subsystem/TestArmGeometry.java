package frc.robot.subsystem;

import frc.robot.subsystems.ArmGeometry;
import org.a05annex.util.AngleD;
import org.a05annex.util.Utl;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import java.awt.geom.Point2D;

import static org.junit.jupiter.api.Assertions.assertEquals;


public class TestArmGeometry {
    @Test
    @DisplayName("test EXTENSION_TICS_PER_INCH")
    void TestExtensionTicsPerInch() {
        System.out.println("---------------------------------------------------------------------------------");
        System.out.println(String.format("Extension tics per inch = %.4f", ArmGeometry.EXTENSION_TICS_PER_INCH));
    }

    @Test
    @DisplayName("test conversions Arm Length to/from Encoder Position")
    void TestLengthEncoderPositionConversion() {
        System.out.println("---------------------------------------------------------------------------------");
        // verify minimum and maximum extension
        assertEquals(ArmGeometry.ARM_LENGTH_MIN_EXTENSION,
                ArmGeometry.getArmLengthFromExtensionPosition(ArmGeometry.EXTENSION_POSITION_MIN_EXTENSION), 0.0001);
        assertEquals(ArmGeometry.ARM_LENGTH_MAX_EXTENSION,
                ArmGeometry.getArmLengthFromExtensionPosition(ArmGeometry.EXTENSION_POSITION_MAX_EXTENSION), 0.0001);
        assertEquals(ArmGeometry.EXTENSION_POSITION_MIN_EXTENSION,
                ArmGeometry.getExtensionPositionFromArmLength(ArmGeometry.ARM_LENGTH_MIN_EXTENSION), 0.0001);
        assertEquals(ArmGeometry.EXTENSION_POSITION_MAX_EXTENSION,
                ArmGeometry.getExtensionPositionFromArmLength(ArmGeometry.ARM_LENGTH_MAX_EXTENSION), 0.0001);
        // run a sequence of encode positions from minimum tp maximum and verify a round-trip from position
        // to length to position gets back to the same position.
        for (double encoderPosition = ArmGeometry.EXTENSION_POSITION_MIN_EXTENSION;
             ArmGeometry.EXTENSION_POSITION_MAX_EXTENSION < ArmGeometry.EXTENSION_POSITION_MIN_EXTENSION ?
                     encoderPosition >= ArmGeometry.EXTENSION_POSITION_MAX_EXTENSION :
                     encoderPosition <= ArmGeometry.EXTENSION_POSITION_MAX_EXTENSION;
             encoderPosition +=
                     ArmGeometry.EXTENSION_POSITION_MAX_EXTENSION < ArmGeometry.EXTENSION_POSITION_MIN_EXTENSION ?
                             -10.0 : 10.0) {
            double positionToLength = ArmGeometry.getArmLengthFromExtensionPosition(encoderPosition);
            double lengthToPosition = ArmGeometry.getExtensionPositionFromArmLength(positionToLength);
            System.out.println(String.format("Extension Position = %10.2f; to length = %10.3f; back to encoder %10.3f",
                    encoderPosition, positionToLength, lengthToPosition));
            assertEquals(encoderPosition, lengthToPosition, 0.00001);
        }
    }

    @Test
    @DisplayName("test conversions pivot angle to/from Encoder Position")
    void TestPivotEncoderAngleConversion() {
        System.out.println("---------------------------------------------------------------------------------");
        // Run a sequence of encoder positions from minimum to maximum and verify round trip conversions get
        // back to the same encoder values.
        for (double pivotPosition = ArmGeometry.PIVOT_MIN_POSITION;
             pivotPosition <= ArmGeometry.PIVOT_MAX_POSITION; pivotPosition += 5.0) {
            AngleD positionToAngle = ArmGeometry.getPivotAngleFromPosition(pivotPosition);
            double angleToPosition = ArmGeometry.getPivotPositionFromAngle(positionToAngle);
            System.out.println(String.format("Pivot Position = %10.2f; to angle = %10.3f; back to position %10.3f",
                    pivotPosition, positionToAngle.getDegrees(), pivotPosition));
            assertEquals(pivotPosition, pivotPosition, 0.00001);
        }
    }

    private void locationToFromPosition(double x, double y) {
        ArmGeometry.ArmPositions positions = ArmGeometry.getArmPositionsFromLocation(x, y);
        Point2D.Double location = ArmGeometry.getArmLocationFromPositions(
                positions.getPivotPosition(), positions.getExtensionPosition());
        System.out.println(String.format(
                "location: (%7.2f,%7.2f); positions: (%7.2f,%7.2f); back to location: (%7.2f,%7.2f)",
                x, y, positions.getPivotPosition(), positions.getExtensionPosition(),
                location.x, location.y ));
        assertEquals(x, location.x, 0.00001);
        assertEquals(y, location.y, 0.00001);
    }
    @Test
    @DisplayName("test location to/from positions")
    void TestLocationToFromPositions() {
        double x, y;
        System.out.println("---------------------------------------------------------------------------------");
        System.out.println("TestLocationToFromPositions - no clipping for valid extent ");
        System.out.println("---------------------------------------------------------------------------------");
        // run a sequence from (0",0") to (90",0") by 5" increments
        for (x = 0, y = 0; x <= 90.0; x += 5.0) {
            locationToFromPosition(x,y);
        }
        System.out.println("---------------------------------------------------------------------------------");
        // run a sequence from (0",30") to (90",0") by 5" increments
        for (x = 0, y = 30; x <= 90.0; x += 5.0) {
            locationToFromPosition(x,y);
        }
        System.out.println("---------------------------------------------------------------------------------");
        // run a sequence from (0",60") to (90",0") by 5" increments
        for (x = 0, y = 60; x <= 90.0; x += 5.0) {
            locationToFromPosition(x,y);
        }
        System.out.println("---------------------------------------------------------------------------------");
        // run a sequence from (50",-30") to (50",120") by 5" increments
        for (x = 50, y = -30; y <= 120.0; y += 5.0) {
            locationToFromPosition(x,y);
       }
        System.out.println("---------------------------------------------------------------------------------");
        // run a sequence from (20",-30") to (20",120") by 5" increments
        for (x = 20, y = -30; y <= 120.0; y += 5.0) {
            locationToFromPosition(x,y);
        }
    }

    @Test
    @DisplayName("test clipping, by doing an angle sweep with invalid long length")
    void TestClippingBySweep() {
        System.out.println("---------------------------------------------------------------------------------");
        System.out.println("TestClippingBySweep");
        System.out.println("---------------------------------------------------------------------------------");
        for (double degrees = -5.0; degrees <= 150.0; degrees += 5.0) {
            AngleD pivotAngle = new AngleD().setDegrees(degrees);
            double x = 120.0 * pivotAngle.sin();
            double y = 120.0 * pivotAngle.cos();
            ArmGeometry.ArmPositions positions = ArmGeometry.getArmPositionsFromLocation(x, y);
            boolean clipped = positions.clipToValidInPlay();
            Point2D.Double location = ArmGeometry.getArmLocationFromPositions(
                    positions.getPivotPosition(), positions.getExtensionPosition());
            System.out.println(String.format(
                    "location: (%7.2f,%7.2f); positions: (%7.2f,%7.2f) - valid:%b; back to location: (%7.2f,%7.2f)",
                    x, y, positions.getPivotPosition(), positions.getExtensionPosition(), clipped,
                    location.x, location.y ));
        }
    }
    @Test
    @DisplayName("test old clipping, by doing an angle sweep.")
    void TestOldClippingBySweep() {
        System.out.println("---------------------------------------------------------------------------------");
        System.out.println("TestOldClippingBySweep");
        System.out.println("---------------------------------------------------------------------------------");
        for (double degrees = -5.0; degrees <= 150.0; degrees += 5.0) {
            AngleD pivotAngle = new AngleD().setDegrees(degrees);
            double pivotPosition = ArmGeometry.getPivotPositionFromAngle(pivotAngle);
            // this is the computation of maximum arm extension position from Ethan's
            // original code.
            double distInches = 39.5 / pivotAngle.sin();
            double extensionPosition = 200 - (distInches * 3.377 * 0.9);
            extensionPosition = Utl.clip(extensionPosition, 0, 210);
            // get the 2d location so the path can be graphed.
            Point2D.Double location = ArmGeometry.getArmLocationFromPositions(
                    pivotPosition, extensionPosition);
            System.out.println(String.format(
                    "angle(deg): %7.2f; positions: (%7.2f,%7.2f); back to location: (%7.2f,%7.2f)",
                    degrees, pivotPosition, extensionPosition, location.x, location.y));
        }
    }


}
