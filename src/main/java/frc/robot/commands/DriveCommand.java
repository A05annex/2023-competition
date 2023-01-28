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
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

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
        m_photonVisionSubsystem.updateLastTarget(Constants.DRIVE_CAMERA);

        //Constants.updateConstant("area alpha", m_photonVisionSubsystem.areaOffsetAverageAlpha);
        //Constants.updateConstant("yaw alpha", m_photonVisionSubsystem.yawOffsetAverageAlpha);

        //SmartDashboard.putNumber("areaCalc", -Utl.clip((Utl.clip(m_photonVisionSubsystem.getAreaOffsetAverage(m_photonVisionSubsystem.lastResult), 0.0, 6.0)-3.0)/6.0, -1.0, 1.0));
        //SmartDashboard.putBoolean("hasTarget", m_photonVisionSubsystem.lastResult.hasTargets());

        if(m_driveXbox.getAButton() && m_photonVisionSubsystem.hasTarget(m_photonVisionSubsystem.lastResult)) {
            m_conditionedDirection.atan2(Utl.clip(m_photonVisionSubsystem.getYawOffsetAverage(m_photonVisionSubsystem.lastResult)/30, -1.0, 1.0),
                    -Utl.clip((Utl.clip(m_photonVisionSubsystem.getAreaOffsetAverage(m_photonVisionSubsystem.lastResult), 0.0, 6.0)-3)/3, -1.0, 1.0));

            double speedDistance = Math.pow(Utl.clip(Math.abs(m_photonVisionSubsystem.getYawOffsetAverage(m_photonVisionSubsystem.lastResult)/30), 0.0, 1.0), 1.75) +
                    Math.pow(Utl.clip(Math.abs((Utl.clip(m_photonVisionSubsystem.getAreaOffsetAverage(m_photonVisionSubsystem.lastResult), 0.0, 6.0)-3.0)/3.0), 0.0, 1.0), 1.75);
            m_conditionedSpeed = Utl.clip(speedDistance, m_lastConditionedSpeed - m_driver.getDriveSpeedMaxInc(),
                    m_lastConditionedSpeed + m_driver.getDriveSpeedMaxInc());
            m_lastConditionedSpeed = m_conditionedSpeed;
        } else if(m_driveXbox.getAButton()) {
            m_conditionedSpeed = m_lastConditionedSpeed;
            m_conditionedDirection = m_lastConditionedDirection;
        } else {
            conditionStick();
        }

        m_driveSubsystem.swerveDrive(m_conditionedDirection, m_conditionedSpeed, m_conditionedRotate);
    }
}