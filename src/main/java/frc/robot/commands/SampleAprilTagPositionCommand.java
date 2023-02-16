package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.PhotonVisionSubsystem;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.commands.A05DriveCommand;
import org.a05annex.frc.subsystems.DriveSubsystem;
import org.a05annex.util.AngleConstantD;
import org.a05annex.util.Utl;


public class SampleAprilTagPositionCommand extends A05DriveCommand {

    private final PhotonVisionSubsystem m_photonSubsystem = PhotonVisionSubsystem.getInstance();
    private final DriveSubsystem m_driveSubsystem = DriveSubsystem.getInstance();

    private final double maxSpeedDelta = 0.075;

    // puts movement to the power of this var
    private final double speedSmoothingMultiplier = 1.75;

    // changing this will make the
    private final double yawOffset = 0.0, areaOffset = 0.0;

    private final double yawThreshold = 0.0, areaThreshold = 0.0;

    private int ticksAligned = 0;
    private boolean alignedWithAprilTag = false;

    public SampleAprilTagPositionCommand(XboxController xbox, A05Constants.DriverSettings driver) {
        super(xbox, driver);
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(m_driveSubsystem);
    }

    @Override
    public void initialize() {
        m_lastConditionedDirection.setRadians(0.0);
        m_lastConditionedSpeed = 0.0;
        m_lastConditionedRotate = 0.0;
        alignedWithAprilTag = false;
        ticksAligned = 0;
        m_driveSubsystem.setHeading(AngleConstantD.ZERO);
    }

    @Override
    public void execute() {
        // checks to see if we have aligned with an AprilTag yet.
        if (!alignedWithAprilTag) {
            // grab last target. Prevents a new frame that could be missing a target from coming in until everything has run
            m_photonSubsystem.updateLastTarget(Constants.DRIVE_CAMERA);
            m_photonSubsystem.calcYawOffsetAverage(m_photonSubsystem.lastTargetFrame, -30.0, 30.0, yawOffset);
            m_photonSubsystem.calcAreaOffsetAverage(m_photonSubsystem.lastTargetFrame, 0.0, 6.0, areaOffset);

            // was there was a target in the last frame
            if(m_photonSubsystem.lastTargetFrame.hasTargets()) {

                // Get the ATan2 of the yaw offset and the area offset to calculate a direction to drive in
                // Uses methods to smooth area and yaw to account for indecisive vision processing
                m_conditionedDirection.atan2(m_photonSubsystem.getYawOffsetAverage(-30.0, 30.0, yawOffset),
                        -m_photonSubsystem.getAreaOffsetAverage(0.0, 6.0, areaOffset));

                // Find how fast to move the robot (value between 0.0 - 1.0)
                // puts both speeds to a power greater than 1 to slow down the robot as it closes in (speedSmoothingMultiplier)
                // Uses methods to smooth area and yaw to account for indecisive vision processing
                double speedDistance = Math.pow(Math.abs(m_photonSubsystem.getYawOffsetAverage(-30.0, 30.0, yawOffset)), speedSmoothingMultiplier) +
                        Math.pow(Math.abs(m_photonSubsystem.getAreaOffsetAverage(0.0, 6.0, areaOffset)), speedSmoothingMultiplier);

                // We apply a speed change limit to swerve to prevent burnouts and smooth robot movements
                // value only changes by at most DriveSpeedMaxInc
                m_conditionedSpeed = Utl.clip(speedDistance, m_lastConditionedSpeed - maxSpeedDelta,
                        m_lastConditionedSpeed + maxSpeedDelta);

                m_conditionedSpeed = Utl.clip(m_conditionedSpeed, 0.0, 0.25);

                //update lastConditionedSpeed
                m_lastConditionedSpeed = m_conditionedSpeed;
                m_lastConditionedDirection = m_conditionedDirection;
            }
            // if the A button is pressed but there was not a target in the last frame
            else {
                // Drive using the last set speeds
                m_conditionedSpeed = 0;
                m_conditionedDirection = m_lastConditionedDirection;
            }

            // Passes direction, speed, and rotation from above into the swerveDrive method which actually spins the wheels
            m_driveSubsystem.swerveDrive(m_conditionedDirection, m_conditionedSpeed, m_conditionedRotate);

            // Is the robot close enough to where it should be?
            if (Math.abs(m_photonSubsystem.getYawOffsetAverage()) < yawThreshold && Math.abs(m_photonSubsystem.getAreaOffsetAverage()) < areaThreshold) {
                // Yes? add 1 to the counter
                ticksAligned++;
                if (ticksAligned >= 25) { // 25 ticks. 25 * 20 ms = 0.5 seconds
                    alignedWithAprilTag = true;
                }
            }
            else {
                // No? reset the counter
                ticksAligned = 0;
            }
        }
        else {
            //m_driveSubsystem.translate(0.0, 1.0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
