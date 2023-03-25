package frc.robot.subsystems;

import org.a05annex.frc.NavX;
import org.a05annex.frc.subsystems.DriveSubsystem;
import org.a05annex.frc.subsystems.ISwerveDrive;
import org.a05annex.util.AngleConstantD;
import org.a05annex.util.AngleD;

public class SpeedCachedSwerve implements ISwerveDrive {
    private final static SpeedCachedSwerve INSTANCE = new SpeedCachedSwerve();

    public static SpeedCachedSwerve getInstance() {
        return INSTANCE;
    }

    static class ControlRequest {
        double forward;
        double strafe;
        double rotation;
        long time;
    }

    ControlRequest[] controlRequests = null;

    int cacheLength;
    int mostRecentControlRequest;

    private DriveMode driveMode = DriveMode.FIELD_RELATIVE;

    private DriveSubsystem driveSubsystem = null;

    public SpeedCachedSwerve() {

    }

    //public void getMostRecentControlRequest

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

    public void setDriveSubsystem(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        if (driveSubsystem != null) {
            driveMode = driveSubsystem.getDriveMode();
        }
    }

    public DriveSubsystem getDriveSubsystem() {
        return driveSubsystem;
    }

    @Override
    public void setDriveGeometry(double v, double v1, double v2, double v3, double v4, double v5, double v6) {
        if (driveSubsystem != null) {
            driveSubsystem.setDriveGeometry(v, v1, v2, v3, v4, v5, v6);
        }
    }

    @Override
    public double getDriveLength() {
        if (driveSubsystem != null) {
            return driveSubsystem.getDriveLength();
        }
        return 0;
    }

    @Override
    public double getDriveWidth() {
        if (driveSubsystem != null) {
            return driveSubsystem.getDriveWidth();
        }

        return 0;
    }

    @Override
    public double getMaxMetersPerSec() {
        if (driveSubsystem != null) {
            return driveSubsystem.getMaxMetersPerSec();
        }

        return 0;
    }

    @Override
    public double getMaxRadiansPerSec() {
        if (driveSubsystem != null) {
            return driveSubsystem.getMaxRadiansPerSec();
        }

        return 0;
    }

    @Override
    public void setFieldPosition(double v, double v1, AngleD angleD) {
        if (driveSubsystem != null) {
            driveSubsystem.setFieldPosition(v, v1, angleD);
        }
    }

    @Override
    public void swerveDriveComponents(double forward, double strafe, double rotation) {
        if (driveSubsystem != null) {
            driveSubsystem.swerveDriveComponents(forward, strafe, rotation);
        }

        mostRecentControlRequest++;
        if(mostRecentControlRequest >= cacheLength) {
            mostRecentControlRequest = 0;
        }

        controlRequests[mostRecentControlRequest].forward = forward;
        controlRequests[mostRecentControlRequest].strafe = strafe;
        controlRequests[mostRecentControlRequest].rotation = rotation;
        controlRequests[mostRecentControlRequest].time = System.currentTimeMillis();
    }

    @Override
    public void prepareForDriveComponents(double v, double v1, double v2) {
        if (driveSubsystem != null) {
            driveSubsystem.prepareForDriveComponents(v, v1, v2);
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
