// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.PhotonVisionSubsystem;
import org.a05annex.frc.A05Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.PhotonCamera;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants extends A05Constants
{
    public static final class CAN_Devices {
        public static final int
                // Non-Drive Motors
                PIVOT_FORWARD_SUPPORT_MOTOR = 12,
                PIVOT_BACKWARD_SUPPORT_MOTOR = 11,
                ARM_EXTENSION_MOTOR = 13,

                CONE_MOTOR = 9,
                CUBE_MOTOR = 14;
    }

    public enum AprilTagSet {
        SUBSTATION(new int[] {4}, new int[] {5}, false),
        NODE(new int[] {6, 7, 8}, new int[] {1, 2, 3}, true);

        public final int[] blue;
        public final int[] red;
        public final boolean upfield;

        AprilTagSet(int[] blue, int[] red, boolean upfield) {
            this.blue = blue;
            this.red = red;
            this.upfield = upfield;
        }
    }

    public static final int PNEUMATICS_FORWARD = 14, PNEUMATICS_REVERSE = 15;

    // Port 0 is whichever controller was plugged in first, not a specific port.
    //DRIVE_XBOX_PORT = 0 (Set in A05Constants) Leave commented
    public static final int ALT_XBOX_PORT = 1; // 2nd Controller for more controls

    //public static final PhotonCamera DRIVE_CAMERA = new PhotonCamera("Camera 1");
    public static final PhotonVisionSubsystem.Camera DRIVE_CAMERA = new PhotonVisionSubsystem.Camera(new PhotonCamera("IMX219"), PhotonVisionSubsystem.PIPELINE.APRILTAGS);
    //public static final PhotonCamera CLAW_CAMERA = new PhotonCamera("Camera 2");

    // kP for keeping drive at the same orientation
    public static double DRIVE_ORIENTATION_kP = 1.2;

    // for practice, length and width from center of the wheels, in m (note chassis is 30" square,
    // the bolt pattern is 29" square, wheels are 2.75" in from the bolt pattern or centered on the
    // corners of a 23.5"(0.5969m) square.
    // For competition, length and width from center of the wheels, in m (note chassis is 28" square,
    // the bolt pattern is 27" square, wheels are 2.75" in from the bolt pattern or centered on the
    // corners of a 21.5"(0.5461m) square.
    /**
     * The geometry and calibration specific to a swerve drive robot base. We currently have 2 bases, the first being
     * a <i>prototyping/practice</i> base that should always be in working condition for drive tuning/testing,
     * calibration, as well as software prototyping. The second is the <i>competition</i> robot that is built for
     * the yearly competition, and is generally not drivable between the introduction of the competition and a
     * week or two before the first competition because all the competition-specific appendages are being built
     * and assembled to it.
     *
     * The <i>competition</i> robot is the index 0 entry in the list, so it does not require any changes to the
     * Roborio. The <i>practice</i> robot requires a jumper on the digital input port 5 that connects the signal
     * pin to ground. This convention was chosen to minimize the things that could go wrong on the competition robot.
     *
     * These settings are loaded into {@link #ROBOT_SETTINGS_LIST} during {@link Robot#robotInit()}
     */
    public static final A05Constants.RobotSettings[] ROBOT_SETTINGS = {
            new A05Constants.RobotSettings(0, "Competition", 0.5461, 0.5461, 2.700, 1.161,
                    2.723, 2.448, 1.026,0.9650),
            new A05Constants.RobotSettings(1, "Practice", 0.5969, 0.5969, 5.240, 5.654,
                    0.969, 5.039, 1.026, 0.9164)
    };

    public static final A05Constants.AutonomousPath[] AUTONOMOUS_PATHS = {
            new A05Constants.AutonomousPath("Balancer", 0, "middleBalance.json"),
            new A05Constants.AutonomousPath("45 offset balance", 1, "45BalanceOffset.json"),
            new A05Constants.AutonomousPath("Side Auto", 2, "side.json"),
            new A05Constants.AutonomousPath("sub side 1", 3, "subScoreAndPickup.json"),
            new A05Constants.AutonomousPath("sub side 2", 4, "subDoubleScore.json")
    };

    public static final A05Constants.DriverSettings[] DRIVER_SETTINGS = {
            new A05Constants.DriverSettings("programmer", 0),
            new A05Constants.DriverSettings("ocean", 1),
            new A05Constants.DriverSettings("hayden", 2)
    };

    private static final DigitalInput switch6 = new DigitalInput(6);

    public static Boolean readMirrorSwitch() {
        return switch6.get();
    }

    // Connect values to SmartDashboard, if you change the value in smart dashboard it changes the const
    // (speed adjusting etc.) By having two methods, you can optionally add the bounds
    /**
     * Initialize value on SmartDashboard for user input, or if already present, return current value.
     *
     * @param key (String) The key to associate with the value.
     * @param initValue (double) The default value to assign if not already on SmartDashboard.
     *
     * @return The new value that appears on the dashboard.
     */
    @SuppressWarnings("unused")
    public static double updateConstant(String key, double initValue) {
        // if key already exists, value will be the current value or whatever we just typed in to the dashboard
        // if key doesn't exist yet, value will be set to initValue and added to SmartDashboard
        double value = SmartDashboard.getNumber(key, initValue);

        // add number if it doesn't exist, or just set it to its current value
        SmartDashboard.putNumber(key, value);
        return value;
    }

    /**
     * Initialize value on SmartDashboard for user input, or if already present, return current value.
     * If value is outside (lowerBound, upperBound), it will be set to the previous value.
     *
     * @param key (String) The key to associate with the value.
     * @param initValue (double) The default value to assign if not already on SmartDashboard.
     * @param lowerBound (double) Lower bound on the value.
     * @param upperBound (double) Upper bound on the value.
     *
     * @return The new value that appears on the dashboard.
     */
    @SuppressWarnings("unused")
    public static double updateConstant(String key, double initValue, double lowerBound, double upperBound) {
        // if key already exists, value will be the current value or whatever we just typed in to the dashboard
        // if key doesn't exist yet, value will be set to initValue and added to SmartDashboard
        double value = SmartDashboard.getNumber(key, initValue);

        // bounds check
        if (value < lowerBound || value > upperBound) {
            value = initValue;
        }

        // add number if it doesn't exist, or just set it to its current value
        SmartDashboard.putNumber(key, value);
        return value;
    }
}
