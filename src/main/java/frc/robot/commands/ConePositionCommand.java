package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SpeedCachedSwerve;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.commands.A05DriveCommand;
import org.a05annex.frc.subsystems.DriveSubsystem;
import org.a05annex.util.AngleConstantD;
import org.a05annex.util.AngleD;
import org.a05annex.util.AngleUnit;
import org.a05annex.util.Utl;
import org.photonvision.targeting.PhotonPipelineResult;


public class ConePositionCommand extends A05DriveCommand {

    private final PhotonVisionSubsystem m_photonSubsystem = PhotonVisionSubsystem.getInstance();

    private PhotonPipelineResult lastFrame;

    // Biggest speed change that can happen in one tick
    private final double maxSpeedDelta = 0.075;
    // Max speed regardless of what the calculation finds
    private final double maxSpeed = 0.5;

    // puts movement to the power of this var
    private final double speedSmoothingMultiplier = 1.4;

    private final double yawMin = -24.5, yawMax = 24.5;
    private final double areaMin = 0.0, areaMax = 5;

    // What target values should the robot try to drive to
    private final double yawOffset = 9.47, areaOffset = 1.7;

    //
    private final double yawThreshold = 0.04, areaThreshold = 0.1;

    private int ticksAligned = 0;
    private boolean alignedWithAprilTag = false;

    private boolean isFinished = false;

    public ConePositionCommand(XboxController xbox, A05Constants.DriverSettings driver) {
        // NOTE: the super adds the drive subsystem requirement
        super(SpeedCachedSwerve.getInstance(), xbox, driver);
    }

    @Override
    public void initialize() {
        m_lastConditionedDirection = new AngleD(AngleUnit.RADIANS, 0.0);
        m_lastConditionedSpeed = 0.0;
        m_lastConditionedRotate = 0.0;
        alignedWithAprilTag = false;
        isFinished = false;
        ticksAligned = 0;
        lastFrame = Constants.DRIVE_CAMERA.getCamera().getLatestResult();
    }

    @Override
    public void execute() {
        // checks to see if we have aligned with an AprilTag yet.
        if (!alignedWithAprilTag) {
            // grab last target. Prevents a new frame that could be missing a target from coming in until everything has run
            lastFrame = m_photonSubsystem.updateLastTarget(Constants.DRIVE_CAMERA.getCamera(), lastFrame);

            // was there was a target in the last frame
            if(lastFrame.hasTargets()) {
                m_photonSubsystem.calcYawOffsetAverage(lastFrame, yawMin, yawMax);
                m_photonSubsystem.calcAreaOffsetAverage(lastFrame, areaMin, areaMax);

                // Get the atan2 of the yaw offset and the area offset to calculate a direction to drive in
                // Uses methods to smooth area and yaw to account for indecisive vision processing
                m_conditionedDirection.atan2(m_photonSubsystem.getYawOffsetAverage(yawMin, yawMax, yawOffset),
                        -m_photonSubsystem.getAreaOffsetAverage(areaMin, areaMax, areaOffset));

                // Add pi because you are moving up field
                m_conditionedDirection.add(AngleConstantD.PI);

                // Find how fast to move the robot (value between 0.0 - 1.0)
                // puts both speeds to a power greater than 1 to slow down the robot as it closes in (speedSmoothingMultiplier)
                // Uses methods to smooth area and yaw to account for indecisive vision processing
                double speedDistance = Utl.length(Math.pow(Math.abs(m_photonSubsystem.getYawOffsetAverage(yawMin, yawMax, yawOffset)), speedSmoothingMultiplier),
                        Math.pow(Math.abs(m_photonSubsystem.getAreaOffsetAverage(areaMin, areaMax, areaOffset)), speedSmoothingMultiplier));

                // We apply a speed delta limit to swerve to prevent burnouts and smooth robot movements
                // value only changes by at most maxSpeedDelta
                m_conditionedSpeed = Utl.clip(speedDistance, m_lastConditionedSpeed - maxSpeedDelta,
                        m_lastConditionedSpeed + maxSpeedDelta);

                // Limit speed to make sure the camera can always see the AprilTag
                m_conditionedSpeed = Utl.clip(m_conditionedSpeed, 0.0, maxSpeed);
            }
            //there was not a target in the last frame
            else {
                // Drive using the last set speeds
                m_conditionedSpeed = 0.0;
                m_conditionedDirection = new AngleD(AngleUnit.RADIANS, 0.0);
            }

            m_lastConditionedSpeed = m_conditionedSpeed;
            m_lastConditionedDirection = m_conditionedDirection;

            m_navx.setExpectedHeading(m_navx.getHeadingInfo().getClosestUpField());
            m_conditionedRotate = new AngleD(m_navx.getHeadingInfo().expectedHeading).subtract(new AngleD(m_navx.getHeadingInfo().heading))
                    .getRadians() * A05Constants.getDriveOrientationkp();

            // Passes direction, speed, and rotation from above into the swerveDrive method which actually spins the wheels
            iSwerveDrive.swerveDrive(m_conditionedDirection, m_conditionedSpeed, m_conditionedRotate);

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
            // You were in the threshold for long enough, stop the swerve and end the command
            iSwerveDrive.swerveDrive(AngleConstantD.ZERO, 0.0, 0.0);
            isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
