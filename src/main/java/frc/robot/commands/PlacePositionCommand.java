package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.PhotonVisionSubsystem;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.NavX;
import org.a05annex.frc.commands.A05DriveCommand;
import org.a05annex.frc.subsystems.DriveSubsystem;
import org.a05annex.util.AngleConstantD;
import org.a05annex.util.AngleD;
import org.a05annex.util.AngleUnit;
import org.a05annex.util.Utl;
import org.photonvision.targeting.PhotonPipelineResult;


public class PlacePositionCommand extends A05DriveCommand {

    private final PhotonVisionSubsystem m_photonSubsystem = PhotonVisionSubsystem.getInstance();
    private final DriveSubsystem m_driveSubsystem = DriveSubsystem.getInstance();

    private PhotonPipelineResult lastFrame;

    private final double maxSpeedDelta = 0.075;

    // puts movement to the power of this var
    private final double speedSmoothingMultiplier = 1.6;

    // changing this will make the
    private final double yawOffset = 4.48, areaOffset = 12.0;

    private final double yawThreshold = 0.025, areaThreshold = 0.1;

    private int ticksAligned = 0;
    private boolean alignedWithAprilTag = false;

    public PlacePositionCommand(XboxController xbox, A05Constants.DriverSettings driver) {
        super(xbox, driver);
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(m_driveSubsystem);
    }

    @Override
    public void initialize() {
        m_lastConditionedDirection = new AngleD(AngleUnit.RADIANS, 0.0);
        m_lastConditionedSpeed = 0.0;
        m_lastConditionedRotate = 0.0;
        alignedWithAprilTag = false;
        ticksAligned = 0;
        m_driveSubsystem.setHeading(AngleConstantD.PI);
        lastFrame = Constants.DRIVE_CAMERA.getLatestResult();
    }

    @Override
    public void execute() {
        // checks to see if we have aligned with an AprilTag yet.
        if (!alignedWithAprilTag) {
            // grab last target. Prevents a new frame that could be missing a target from coming in until everything has run
            lastFrame = m_photonSubsystem.updateLastTarget(Constants.DRIVE_CAMERA, lastFrame);

            // was there was a target in the last frame
            if(lastFrame.hasTargets()) {
                m_photonSubsystem.calcYawOffsetAverage(lastFrame, -7.5, 7.5);
                m_photonSubsystem.calcAreaOffsetAverage(lastFrame, 0.0, 14.0);

                // Get the ATan2 of the yaw offset and the area offset to calculate a direction to drive in
                // Uses methods to smooth area and yaw to account for indecisive vision processing
                m_conditionedDirection.atan2(m_photonSubsystem.getYawOffsetAverage(-7.5, 7.5, yawOffset),
                        -m_photonSubsystem.getAreaOffsetAverage(0.0, 14.0, areaOffset));

                m_conditionedDirection.add(AngleConstantD.DEG_180);

                // Find how fast to move the robot (value between 0.0 - 1.0)
                // puts both speeds to a power greater than 1 to slow down the robot as it closes in (speedSmoothingMultiplier)
                // Uses methods to smooth area and yaw to account for indecisive vision processing
                double speedDistance = Utl.length(Math.pow(Math.abs(m_photonSubsystem.getYawOffsetAverage(-7.5, 7.5, yawOffset)), speedSmoothingMultiplier),
                        Math.pow(Math.abs(m_photonSubsystem.getAreaOffsetAverage(0.0, 14.0, areaOffset)), speedSmoothingMultiplier));

                // We apply a speed change limit to swerve to prevent burnouts and smooth robot movements
                // value only changes by at most DriveSpeedMaxInc
                m_conditionedSpeed = Utl.clip(speedDistance, m_lastConditionedSpeed - maxSpeedDelta,
                        m_lastConditionedSpeed + maxSpeedDelta);

                // Limit to half speed to make sure the camera can always see the AprilTag
                m_conditionedSpeed = Utl.clip(m_conditionedSpeed, 0.0, 0.4);

                // Set rotation to be the offset from straight
                if(NavX.getInstance().getNavInfo().yaw.getDegrees() > 0) {
                    m_conditionedRotate = -1.0/180.0 * Math.abs(NavX.getInstance().getNavInfo().yaw.getDegrees()) + 1;
                } else {
                    m_conditionedRotate = (1.0 / 180.0) * Math.abs(NavX.getInstance().getNavInfo().yaw.getDegrees()) - 1;
                }
            }
            // if the A button is pressed but there was not a target in the last frame
            else {
                // Drive using the last set speeds
                m_conditionedSpeed = 0.0;
                m_conditionedDirection = new AngleD(AngleUnit.RADIANS, 0.0);
                //m_conditionedRotate = 0.0;
            }

            m_lastConditionedSpeed = m_conditionedSpeed;
            m_lastConditionedDirection = m_conditionedDirection;
            m_lastConditionedRotate = m_conditionedRotate;

            // Passes direction, speed, and rotation from above into the swerveDrive method which actually spins the wheels
            m_driveSubsystem.swerveDrive(m_conditionedDirection, m_conditionedSpeed, m_conditionedRotate);

            // Is the robot close enough to where it should be?
            if (Math.abs(m_photonSubsystem.getYawOffsetAverage(-30.0, 30.0, yawOffset)) < yawThreshold && Math.abs(m_photonSubsystem.getAreaOffsetAverage(0.0, 7.0, areaOffset)) < areaThreshold) {
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
            m_driveSubsystem.swerveDrive(AngleConstantD.ZERO, 0.0, 0.0);
            SmartDashboard.putBoolean("aligned", alignedWithAprilTag);
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
