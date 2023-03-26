package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.SpeedCachedSwerve;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.commands.A05DriveCommand;

/**
 * Drive command is here because you will likely need to override the serve (targeting, competition specific reason)
 */
public class DriveCommand extends A05DriveCommand {

    private final SpeedCachedSwerve driveSubsystem = SpeedCachedSwerve.getInstance();

    /**
     * Default command for DriveSubsystem. Left stick moves the robot field-relatively, and right stick X rotates.
     * Contains driver constants for sensitivity, gain, and deadband.
     * @param xbox (XboxController) The drive xbox controller.
     */
    public DriveCommand(XboxController xbox, A05Constants.DriverSettings driver) {
        super(xbox, driver);
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.driveSubsystem.getDriveSubsystem());
        // This let' us insert a the speed cache before commands get to the drive.
        setISwerveDrive(driveSubsystem);
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