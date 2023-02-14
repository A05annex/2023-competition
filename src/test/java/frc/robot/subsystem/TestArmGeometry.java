package frc.robot.subsystem;

import frc.robot.subsystems.ArmGeometry;
import org.a05annex.util.AngleD;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;


public class TestArmGeometry {
    @Test
    @DisplayName("test EXTENSION_TICS_PER_INCH")
    void TestExtensionTicsPerInch() {
        System.out.println("---------------------------------------------------------------------------------;");
        System.out.println(String.format("Extension tics per inch = %.4f", ArmGeometry.EXTENSION_TICS_PER_INCH));
    }

    @Test
    @DisplayName("test conversions Arm Length to/from Encoder Position")
    void TestLengthEncoderPositionConversion() {
        System.out.println("---------------------------------------------------------------------------------;");
        // verify minimum and maximum extension
        assertEquals(ArmGeometry.ARM_LENGTH_MINIMUM_EXTENSION,
                ArmGeometry.getArmLengthFromExtensionPosition(ArmGeometry.EXTENSION_POSITION_MINIMUM_EXTENSION), 0.0001);
        assertEquals(ArmGeometry.ARM_LENGTH_MAXIMUM_EXTENSION,
                ArmGeometry.getArmLengthFromExtensionPosition(ArmGeometry.EXTENSION_POSITION_MAXIMUM_EXTENSION), 0.0001);
        assertEquals(ArmGeometry.EXTENSION_POSITION_MINIMUM_EXTENSION,
                ArmGeometry.getExtensionPositionFromArmLength(ArmGeometry.ARM_LENGTH_MINIMUM_EXTENSION), 0.0001);
        assertEquals(ArmGeometry.EXTENSION_POSITION_MAXIMUM_EXTENSION,
                ArmGeometry.getExtensionPositionFromArmLength(ArmGeometry.ARM_LENGTH_MAXIMUM_EXTENSION), 0.0001);
        // run a sequence and verify a round-trip from position to length to position gets back to the same position.
        for (double encoderPosition = ArmGeometry.EXTENSION_POSITION_MINIMUM_EXTENSION;
             ArmGeometry.EXTENSION_POSITION_MAXIMUM_EXTENSION < ArmGeometry.EXTENSION_POSITION_MINIMUM_EXTENSION ?
                     encoderPosition >= ArmGeometry.EXTENSION_POSITION_MAXIMUM_EXTENSION :
                     encoderPosition <= ArmGeometry.EXTENSION_POSITION_MAXIMUM_EXTENSION;
             encoderPosition +=
                     ArmGeometry.EXTENSION_POSITION_MAXIMUM_EXTENSION < ArmGeometry.EXTENSION_POSITION_MINIMUM_EXTENSION ?
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
        System.out.println("---------------------------------------------------------------------------------;");
        for (double pivotPosition = ArmGeometry.PIVOT_MIN_POSITION;
             pivotPosition <= ArmGeometry.PIVOT_MAX_POSITION; pivotPosition += 5.0) {
            AngleD positionToAngle = ArmGeometry.getPivotAngleFromPosition(pivotPosition);
            double angleToPosition = ArmGeometry.getPivotPositionFromAngle(positionToAngle);
            System.out.println(String.format("Pivot Position = %10.2f; to angle = %10.3f; back to position %10.3f",
                    pivotPosition, positionToAngle.getDegrees(), pivotPosition));
            assertEquals(pivotPosition, pivotPosition, 0.00001);
        }
    }

}
