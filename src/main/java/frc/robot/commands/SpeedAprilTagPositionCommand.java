package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SpeedCachedSwerve;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.commands.A05DriveCommand;
import org.a05annex.util.AngleConstantD;
import org.a05annex.util.AngleD;
import org.a05annex.util.Utl;


public class SpeedAprilTagPositionCommand extends A05DriveCommand {

    private long startTime;

    private final SpeedCachedSwerve swerveDrive = SpeedCachedSwerve.getInstance();

    SpeedCachedSwerve.RobotRelativePosition postionAtFrame;
    private final double xPosition, yPosition, maxSpeed, speedSmoothingMultiplier;
    private final boolean upfield;

    private boolean isFinished;

    private final Constants.AprilTagSet aprilTagSet;
    private int[] aprilTagIds;

    private final PhotonVisionSubsystem.Camera camera = Constants.DRIVE_CAMERA;

    private final int resumeDrivingTickThreshold = 500;

    private int ticksWithoutTarget;

    private final int ticksInZone = 10;
    private int ticksInZoneCounter;

    private final double inZoneThreshold;

    // Constants
    private final double X_MAX = 3.0, X_MIN = 0.0, Y_MAX = 1.5, Y_MIN = -1.5, MAX_SPEED_DELTA = 0.075, ROTATION_KP = 0.9;

    public SpeedAprilTagPositionCommand(XboxController xbox, A05Constants.DriverSettings driver,
                                        double xPosition, double yPosition, double maxSpeed,
                                        double speedSmoothingMultiplier, Constants.AprilTagSet aprilTagSet) {
        // NOTE: the super adds the drive subsystem requirement
        super(SpeedCachedSwerve.getInstance(), xbox, driver);

        this.xPosition = xPosition;
        this.yPosition = -yPosition;
        this.maxSpeed = maxSpeed;
        this.speedSmoothingMultiplier = speedSmoothingMultiplier;
        this.upfield = aprilTagSet.upfield;
        this.aprilTagSet = aprilTagSet;

        if(xPosition >= 1) {
            inZoneThreshold = Units.inchesToMeters(0.5 * xPosition);
        } else {
            inZoneThreshold = Units.inchesToMeters(0.5 * Math.pow(xPosition, 2.5));
        }
    }

    @Override
    public void initialize() {
        camera.updateLastFrameAndTarget();
        /*
          Is there a good target?
          Yes: set ticksWithoutTarget to 0
          No: set ticksWithoutTarget to the resumeDrivingTickThreshold because that is the point at which we resume normal driver control

          If there is not immediately a target, the driver can keep going until there is a target, which means the
          robot won't randomly stop meaning we move faster and smoother
        */
        ticksWithoutTarget = camera.doLastFrameAndTargetMatch() ? 0 : resumeDrivingTickThreshold;
        ticksInZoneCounter = 0;
        isFinished = false;

        aprilTagIds = NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").getBoolean(true) ? aprilTagSet.red : aprilTagSet.blue;

        startTime = System.currentTimeMillis();
        //heading = upfield ? m_navx.getHeadingInfo().getClosestUpField() : m_navx.getHeadingInfo().getClosestDownField();
    }

    @Override
    public void execute() {
        // Update the last frame and related values
        camera.updateLastFrameAndTarget();


        boolean goodID = false;

        for (int aprilTagId : aprilTagIds) {
            if (aprilTagId == camera.getLastTarget().getFiducialId()) {
                goodID = true;
            }
        }

        if(!goodID) {
            super.execute();
            return;
        }

        /*
        Is there a new target? (can the camera still pickup an aprilTag)

        No: increment ticksWithoutTarget and if we haven't had a new target for a while resume joystick driving
        */
        if(!camera.doLastFrameAndTargetMatch()) {
            ticksWithoutTarget++;
            if(ticksWithoutTarget > resumeDrivingTickThreshold) {
                // We haven't had a target for a while. we are going to resume driver control
                super.execute();
                return;
            }
        }
        else {
            ticksWithoutTarget = 0;
        }

        postionAtFrame = swerveDrive.getRobotRelativePositionSince(camera.getLastTargetTime());
        if (postionAtFrame.cacheOverrun) {
            isFinished = true;
            return;
        }

        // --------- Calculate Speed ---------
        //double totalSpeed = Math.pow(Math.abs(calcX()), speedSmoothingMultiplier) + Math.pow(Math.abs(calcY()), speedSmoothingMultiplier);
        double totalSpeed = Math.pow(Math.sqrt(Math.pow(Math.abs(calcX()), 2) + Math.pow(Math.abs(calcY()), 2)), speedSmoothingMultiplier);

        // Limit the speed delta
        m_conditionedSpeed = Utl.clip(totalSpeed, m_lastConditionedSpeed - MAX_SPEED_DELTA, m_lastConditionedSpeed + MAX_SPEED_DELTA);

        // Slows the robot down as we go longer without a target. Hopefully allows the robot to "catch" the target again
        m_conditionedSpeed *= ((double) (resumeDrivingTickThreshold - ticksWithoutTarget) / (double)resumeDrivingTickThreshold);

        m_conditionedSpeed = Utl.clip(m_conditionedSpeed, 0.0, maxSpeed);


        // ------- Calculate Rotation --------
        AngleD heading = upfield ? m_navx.getHeadingInfo().getClosestUpField() : m_navx.getHeadingInfo().getClosestDownField();
        m_navx.setExpectedHeading(heading);
        m_conditionedRotate = new AngleD(m_navx.getHeadingInfo().expectedHeading).subtract(new AngleD(m_navx.getHeadingInfo().heading))
                .getRadians() * ROTATION_KP;


        // ------- Calculate Direction -------
        m_conditionedDirection.atan2(calcY(), calcX());

        // Add heading offset
        m_conditionedDirection.add(heading);


        // Update lasts
        m_lastConditionedDirection = m_conditionedDirection;
        m_lastConditionedSpeed = m_conditionedSpeed;
        m_lastConditionedRotate = m_conditionedRotate;

        if(Math.abs(camera.getXFromLastTarget() - postionAtFrame.forward - xPosition) < inZoneThreshold && Math.abs(camera.getYFromLastTarget() - postionAtFrame.strafe - yPosition) < inZoneThreshold) {
            ticksInZoneCounter++;
            swerveDrive.swerveDrive(AngleD.ZERO, 0.0, m_conditionedRotate*0.1);
            if(ticksInZoneCounter > ticksInZone) {
                isFinished = true;
            }
            return;
        }

        swerveDrive.swerveDrive(m_conditionedDirection, m_conditionedSpeed, m_conditionedRotate);
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the swerve
        swerveDrive.swerveDrive(AngleConstantD.ZERO, 0.0, 0.0);
    }



    /**
     * Calculates the X speed between -1 and 1
     * @return x speed
     */
    private double calcX() {
        double center = (X_MAX + X_MIN) / 2.0;
        double scale = (X_MAX - X_MIN) / 2.0;
        return Utl.clip((camera.getXFromLastTarget() - postionAtFrame.forward - center) / scale  -  (xPosition - center) / scale , -1.0, 1.0);
    }

    /**
     * Calculates the Y speed between -1 and 1
     * @return y speed
     */
    private double calcY() {
        double center = (Y_MAX + Y_MIN) / 2.0;
        double scale = (Y_MAX - Y_MIN) / 2.0;
        return Utl.clip((camera.getYFromLastTarget() - postionAtFrame.strafe - center) / scale  -  (yPosition - center) / scale, -1.0, 1.0);
    }
}
