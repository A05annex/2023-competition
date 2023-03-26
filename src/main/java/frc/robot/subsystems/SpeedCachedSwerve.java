package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Timer;
import org.a05annex.frc.NavX;
import org.a05annex.frc.subsystems.DriveSubsystem;
import org.a05annex.frc.subsystems.ISwerveDrive;
import org.a05annex.frc.subsystems.Mk4NeoModule;
import org.a05annex.util.AngleConstantD;
import org.a05annex.util.AngleD;
import org.a05annex.util.AngleUnit;
import org.a05annex.util.Utl;

/**
 * This is a layer that goes on top of the swerve drive to provide caching of past drive commands for some
 * period of time so that the relative position of the robot can be approximated for some time in the past. The
 * scenario for use is that there is a sensor system, such as vision processing of a target, that estimates
 * a robot's position relative to the target; however, it has a several command cycle latency. This means
 * that a control algorithm based on the robot's position relative to the target is using information about that
 * position stale by several cycles when used - and needs to be corrected for the robot's current position.
 * <p>
 * This cache provides the data and processing to predict where the robot is now, relative to the location,
 * location reported for some time in the past.
 */
public class SpeedCachedSwerve implements ISwerveDrive {

    /**
     *
     */
    private final static SpeedCachedSwerve INSTANCE = new SpeedCachedSwerve();

    /**
     *
     * @return
     */
    public static SpeedCachedSwerve getInstance() {
//        Timer.getFPGATimestamp()
        return INSTANCE;
    }

    // -----------------------------------------------------------------------------------------------------------------
    // This is the cache of the last requests to the swerve drive and the processing. Here we cache a collection
    // of timestamped robot move commands
    // -----------------------------------------------------------------------------------------------------------------
    /**
     *
     */
    public static class ControlRequest {
        double forward;
        double strafe;
        double rotation;
        double timeStamp;

        void set(double forward, double strafe, double rotation, double timeStamp) {
            this.forward = forward;
            this.strafe = strafe;
            this.rotation = rotation;
            this.timeStamp = timeStamp;
        }

        public double getForward() {
            return forward;
        }
        public double getStrafe() {
            return forward;
        }
        public double getRotation() {
            return forward;
        }
        public double getTimeStamp() {
            return timeStamp;
        }
    }

    public static class RobotRelativePosition {
        public final double forward;
        public final double strafe;
        public final AngleD heading;
        public final double timeStamp;

        RobotRelativePosition(double forward, double strafe, AngleD heading, double timeStamp) {
            this.forward = forward;
            this.strafe = strafe;
            this.heading = heading;
            this.timeStamp = timeStamp;
        }
    }

    ControlRequest[] controlRequests = null;
    int cacheLength;
    int mostRecentControlRequest;

    private DriveSubsystem driveSubsystem = null;
    private DriveMode driveMode = DriveMode.FIELD_RELATIVE;
    private double driveLength = 0.0;
    private double driveWidth = 0.0;
    private double maxMetersPerSec = 3.136;
    private double maxRadiansPerSec = 3.136;

    public SpeedCachedSwerve() {
        // the constructor does nothing ...
    }

    public ControlRequest getMostRecentControlRequest() {
        return controlRequests[mostRecentControlRequest];
    }

    public RobotRelativePosition getRobotRelativePositionSince(double sinceTime) {
        return getRobotRelativePositionSince(Timer.getFPGATimestamp(), sinceTime);
    }

    RobotRelativePosition getRobotRelativePositionSince(double currentTime, double sinceTime) {
        int backIndex = mostRecentControlRequest;
        double forward = 0.0;
        double strafe = 0.0;
        double headingRadians = 0.0;
        while (controlRequests[backIndex].timeStamp > sinceTime) {
            double deltaTime = currentTime - controlRequests[backIndex].timeStamp;
            forward += deltaTime * controlRequests[backIndex].forward * maxMetersPerSec;
            strafe += deltaTime * controlRequests[backIndex].strafe * maxMetersPerSec;
            headingRadians += deltaTime * controlRequests[backIndex].rotation * maxRadiansPerSec;
            currentTime = controlRequests[backIndex].timeStamp;
            backIndex--;
            if (backIndex < 0) {
                backIndex = cacheLength - 1;
            }
        }
        return new RobotRelativePosition(forward,strafe,
                new AngleD(AngleUnit.RADIANS,headingRadians),sinceTime);
    }

    /**
     *
     * @param forward
     * @param strafe
     * @param rotation
     * @param timestamp
     */
    void addControlRequest(double forward, double strafe, double rotation, double timestamp) {
        // increment the index to the array of cached control requests
        mostRecentControlRequest++;
        if(mostRecentControlRequest >= cacheLength) {
            mostRecentControlRequest = 0;
        }
        // save this request with a timestamp
        controlRequests[mostRecentControlRequest].set(forward, strafe, rotation, timestamp);
    }

    public void setCacheLength(int cacheLength) {
        this.cacheLength = cacheLength;
        mostRecentControlRequest = -1;
        controlRequests = new ControlRequest[cacheLength];

        for (int i = 0; i < cacheLength; i++) {
            controlRequests[i] = new ControlRequest();
        }
    }

    public int getCacheLength() {
        return cacheLength;
    }

    // -----------------------------------------------------------------------------------------------------------------
    // The wrapping for the underlying swerve drive subsystem
    // -----------------------------------------------------------------------------------------------------------------
    public void setDriveSubsystem(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        if (driveSubsystem != null) {
            driveMode = driveSubsystem.getDriveMode();
        }
    }

    public DriveSubsystem getDriveSubsystem() {
        return driveSubsystem;
    }

    // -----------------------------------------------------------------------------------------------------------------
    // ISwerveDrive implementation. Most of this is pass-through to the wrapped swerve drive
    // -----------------------------------------------------------------------------------------------------------------
    @Override
    public void setDriveGeometry(double driveLength, double driveWidth,
                                 double rfCalibration, double rrCalibration,
                                 double lfCalibration, double lrCalibration, double maxSpeedCalibration) {
        if (driveSubsystem != null) {
            // There is a drive subsystem - so we get everything we need after the geometry is set.
            driveSubsystem.setDriveGeometry(driveLength, driveWidth,
                    rfCalibration, rrCalibration, lfCalibration, lrCalibration, maxSpeedCalibration);
            this.driveLength = driveSubsystem.getDriveLength();
            this.driveWidth = driveSubsystem.getDriveWidth();
            maxMetersPerSec = driveSubsystem.getMaxMetersPerSec();
            maxRadiansPerSec = driveSubsystem.getMaxRadiansPerSec();
        } else {
            // We are testing, no drive subsystem, so use the geometry to duplicate what would happen
            // in the Drive subsystem
            this.driveLength = driveLength;
            this.driveWidth = driveWidth;
            maxMetersPerSec = Mk4NeoModule.MAX_METERS_PER_SEC * maxSpeedCalibration;
            double driveDiagonal = Utl.length(this.driveLength, this.driveWidth);
            maxRadiansPerSec = maxMetersPerSec / (0.5 * driveDiagonal);
        }
    }

    @Override
    public double getDriveLength() {
        if (driveSubsystem != null) {
            return driveSubsystem.getDriveLength();
        }
        return driveLength;
    }

    @Override
    public double getDriveWidth() {
        if (driveSubsystem != null) {
            return driveSubsystem.getDriveWidth();
        }
        return driveWidth;
    }

    @Override
    public double getMaxMetersPerSec() {
        if (driveSubsystem != null) {
            return driveSubsystem.getMaxMetersPerSec();
        }
        return maxMetersPerSec;
    }

    @Override
    public double getMaxRadiansPerSec() {
        if (driveSubsystem != null) {
            return driveSubsystem.getMaxRadiansPerSec();
        }
        return maxRadiansPerSec;
    }

    @Override
    public void setFieldPosition(double v, double v1, AngleD angleD) {
        if (driveSubsystem != null) {
            driveSubsystem.setFieldPosition(v, v1, angleD);
        }
    }

    @Override
    public void swerveDriveComponents(double forward, double strafe, double rotation) {
        // send the request to the drive if we have one
        if (driveSubsystem != null) {
            driveSubsystem.swerveDriveComponents(forward, strafe, rotation);
        }
        addControlRequest(forward, strafe, rotation, Timer.getFPGATimestamp());
    }

    @Override
    public void prepareForDriveComponents(double forward, double strafe, double rotation) {
        if (driveSubsystem != null) {
            driveSubsystem.prepareForDriveComponents(forward, strafe, rotation);
        }
    }

    @Override
    public void swerveDrive(AngleConstantD direction, double speed, double rotation) {
        if (driveMode == DriveMode.FIELD_RELATIVE) {
            AngleD chassisDirection = new AngleD(direction).subtract(NavX.getInstance().getHeading());
            swerveDriveComponents(chassisDirection.cos() * speed,
                    chassisDirection.sin() * speed, rotation);
        } else {
            swerveDriveComponents(direction.cos() * speed,
                    direction.sin() * speed, rotation);
        }
    }

    @Override
    public void toggleDriveMode() {
        if(driveSubsystem != null) {
            driveSubsystem.toggleDriveMode();
            driveMode = driveSubsystem.getDriveMode();
        }
        else {
            driveMode = (driveMode == DriveMode.FIELD_RELATIVE) ?
                    DriveMode.ROBOT_RELATIVE : DriveMode.FIELD_RELATIVE;
        }
    }

    @Override
    public DriveMode getDriveMode() {
        if(driveSubsystem != null) {
            return driveSubsystem.getDriveMode();
        }

        return driveMode;
    }

    @Override
    public void setDriveMode(DriveMode driveMode) {
        if(driveSubsystem != null) {
            driveSubsystem.setDriveMode(driveMode);
        }
        this.driveMode = driveMode;
    }

    @Override
    public void setHeading(AngleConstantD angleConstantD) {
        if (driveSubsystem != null) {
            driveSubsystem.setHeading(angleConstantD);
        }
    }

    @Override
    public void translate(double v, double v1) {
        if (driveSubsystem != null) {
            driveSubsystem.translate(v, v1);
        }
    }

    @Override
    public void startAbsoluteTranslate(double v, double v1, double v2) {
        if (driveSubsystem != null) {
            driveSubsystem.startAbsoluteTranslate(v, v1, v2);
        }
    }

    @Override
    public void startAbsoluteSmartTranslate(double v, double v1, double v2, double v3) {
        if (driveSubsystem != null) {
            driveSubsystem.startAbsoluteSmartTranslate(v, v1, v2, v3);
        }
    }

    @Override
    public boolean isAbsoluteTranslateDone() {
        if (driveSubsystem != null) {
            return driveSubsystem.isAbsoluteTranslateDone();
        }
        return false;
    }
}
