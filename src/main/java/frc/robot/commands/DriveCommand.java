package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.PhotonVisionSubsystem;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.commands.A05DriveCommand;
import org.a05annex.util.AngleD;
import org.a05annex.util.AngleUnit;
import org.a05annex.util.Utl;

/**
 * Drive command is here because you will likely need to override the serve (targeting, competition specific reason)
 */
public class DriveCommand extends A05DriveCommand {

    private final PhotonVisionSubsystem m_photonVisionSubsystem = PhotonVisionSubsystem.getInstance();

    private A05Constants.DriverSettings m_driver;

    /**
     * Default command for DriveSubsystem. Left stick moves the robot field-relatively, and right stick X rotates.
     * Contains driver constants for sensitivity, gain, and deadband.
     * @param xbox (XboxController) The drive xbox controller.
     */
    public DriveCommand(XboxController xbox, A05Constants.DriverSettings driver) {
        super(xbox, driver);

        m_driver = driver;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        //TODO: If you want to do special control like targeting, comment out super.execute() and add your own control code
        //TODO: Refer to the documentation. Much of the code you want to run is already packaged in callable methods
        //This runs the default swerve calculations for xbox control
        //super.execute();

        conditionStick();

        if(m_driveXbox.getAButton() && m_photonVisionSubsystem.hasTarget(Constants.DRIVE_CAMERA)) {
            m_conditionedDirection = (m_photonVisionSubsystem.getTarget(Constants.DRIVE_CAMERA).getYaw() < 0)
                    ? m_conditionedDirection.setDegrees(180.0) : m_conditionedDirection.setDegrees(0.0);

            double speedDistance = Utl.clip(m_photonVisionSubsystem.getTarget(Constants.DRIVE_CAMERA).getYaw()/30.0, 0.0, 0.5);
            m_conditionedSpeed = Utl.clip(speedDistance, m_lastConditionedSpeed - m_driver.getDriveSpeedMaxInc(),
                    m_lastConditionedSpeed + m_driver.getDriveSpeedMaxInc());
            m_lastConditionedSpeed = m_conditionedSpeed;
        }

        m_driveSubsystem.swerveDrive(m_conditionedDirection, m_conditionedSpeed, m_conditionedRotate);
    }
}