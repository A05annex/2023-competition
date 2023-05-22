package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.SpeedCachedSwerve;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.commands.A05DriveCommand;

/**
 * This {@code DriveCommand} is here because you will likely need to override the default swerve for
 * targeting or other competition specific reason.
 */
public class DriveCommand extends A05DriveCommand {

    /**
     * Default command for DriveSubsystem. Left stick moves the robot field-relatively, and right stick X rotates.
     * Contains driver constants for sensitivity, gain, and deadband.
     * @param xbox (XboxController) The drive xbox controller.
     */
    public DriveCommand(XboxController xbox, A05Constants.DriverSettings driver) {
        // NOTE: the super adds the drive subsystem requirement
        super(SpeedCachedSwerve.getInstance(), xbox, driver);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        //This runs the default swerve calculations for xbox control
        super.execute();
    }
}