package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.subsystems.PhotonVisionSubsystem;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.commands.A05DriveCommand;
import org.a05annex.util.Utl;

/**
 * Drive command is here because you will likely need to override the serve (targeting, competition specific reason)
 */
public class DriveCommand extends A05DriveCommand {

    private final PhotonVisionSubsystem m_photonSubsystem = PhotonVisionSubsystem.getInstance();

    private A05Constants.DriverSettings m_driver;

    private final double speedSmoothingMultiplier = 1.75;

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
        // grab last target. Prevents a new frame that could be missing a target from coming in until everything has run
        m_photonSubsystem.updateLastTarget(Constants.DRIVE_CAMERA);

        // if the A button is pressed and there was a target in the last frame
        if(m_driveXbox.getAButton() && m_photonSubsystem.lastTargetFrame.hasTargets()) {

            // Get the ATan2 of the yaw offset and the area offset to calculate a direction to drive in
            // Uses methods to smooth area and yaw to account for indecisive vision processing
            m_conditionedDirection.atan2(m_photonSubsystem.getScaledCenteredYawOffsetAverage(m_photonSubsystem.lastTargetFrame, -30.0, 30.0),
                    -m_photonSubsystem.getScaledCenteredAreaOffsetAverage(m_photonSubsystem.lastTargetFrame, 0.0, 6.0));

            // Find how fast to move the robot (value between 0.0 - 1.0)
            // puts both speeds to a power greater than 1 to slow down the robot as it closes in (speedSmoothingMultiplier)
            // Uses methods to smooth area and yaw to account for indecisive vision processing
            double speedDistance = Math.pow(Math.abs(m_photonSubsystem.getScaledCenteredYawOffsetAverage(m_photonSubsystem.lastTargetFrame, -30.0, 30.0)), speedSmoothingMultiplier) +
                    Math.pow(Math.abs(m_photonSubsystem.getScaledCenteredAreaOffsetAverage(m_photonSubsystem.lastTargetFrame, 0.0, 6.0)), speedSmoothingMultiplier);

            // We apply a speed change limit to swerve to prevent burnouts and smooth robot movements
            // value only changes by at most DriveSpeedMaxInc
            m_conditionedSpeed = Utl.clip(speedDistance, m_lastConditionedSpeed - m_driver.getDriveSpeedMaxInc(),
                    m_lastConditionedSpeed + m_driver.getDriveSpeedMaxInc());

            //update lastConditionedSpeed
            m_lastConditionedSpeed = m_conditionedSpeed;
        }
        // if the A button is pressed but there was not a target in the last frame
        else if(m_driveXbox.getAButton()) {
            // Drive using the last set speeds
            m_conditionedSpeed = m_lastConditionedSpeed;
            m_conditionedDirection = m_lastConditionedDirection;
        }
        // A button not pressed
        else {
            // runs normal drive with joysticks code
            conditionStick();
        }

        // Passes direction, speed, and rotation from above into the swerveDrive method which actually spins the wheels
        m_driveSubsystem.swerveDrive(m_conditionedDirection, m_conditionedSpeed, m_conditionedRotate);
    }
}