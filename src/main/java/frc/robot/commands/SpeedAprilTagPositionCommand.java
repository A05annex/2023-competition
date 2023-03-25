package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.PhotonVisionSubsystem;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.commands.A05DriveCommand;
import org.a05annex.frc.subsystems.DriveSubsystem;
import org.a05annex.util.AngleConstantD;
import org.a05annex.util.AngleD;
import org.a05annex.util.Utl;


public class SpeedAprilTagPositionCommand extends A05DriveCommand {

    private final DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();

    private final double xPosition, yPosition, maxSpeed, speedSmoothingMultiplier;
    private final AngleD heading;

    private boolean isFinished;

    private final PhotonVisionSubsystem.Camera camera = Constants.DRIVE_CAMERA;

    private final int resumeDrivingTickThreshold = 25;

    private int ticksWithoutTarget;

    private final int ticksInZone = 15;
    private int ticksInZoneCounter;

    private final double inZoneThreshold = Units.inchesToMeters(6.0);

    // Constants
    private final double X_MAX = 4.0, X_MIN = 0.0, Y_MAX = 4.0, Y_MIN = -4.0, MAX_SPEED_DELTA = 0.075;

    public SpeedAprilTagPositionCommand(XboxController xbox, A05Constants.DriverSettings driver, double xPosition, double yPosition, double maxSpeed, double speedSmoothingMultiplier, boolean upfield) {
        super(xbox, driver);

        this.heading = upfield ? m_navx.getHeadingInfo().getClosestUpField() : m_navx.getHeadingInfo().getClosestDownField();
        this.xPosition = xPosition;
        this.yPosition = yPosition;
        this.maxSpeed = maxSpeed;
        this.speedSmoothingMultiplier = speedSmoothingMultiplier;

        isFinished = false;

        ticksInZoneCounter = 0;

        addRequirements(this.driveSubsystem);
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
        ticksWithoutTarget = 0;
    }

    @Override
    public void execute() {
        // Update the last frame and related values
        camera.updateLastFrameAndTarget();
        boolean goodTarget = camera.doLastFrameAndTargetMatch();

        /*
        Is there a new target? (can the camera still pickup an aprilTag)

        No: increment ticksWithoutTarget and if we haven't had a new target for a while resume joystick driving
        */
        if(!goodTarget) {
            ticksWithoutTarget++;
            if(ticksWithoutTarget > resumeDrivingTickThreshold) {
                // We haven't had a target for a while. we are going to resume driver control
                super.execute();
                return;
            }
        }


        // --------- Calculate Speed ---------
        double totalSpeed = Math.pow(Math.abs(calcX()), speedSmoothingMultiplier) + Math.pow(Math.abs(calcY()), speedSmoothingMultiplier);

        // Limit the speed delta
        m_conditionedSpeed = Utl.clip(totalSpeed, m_lastConditionedSpeed - MAX_SPEED_DELTA, m_lastConditionedSpeed + MAX_SPEED_DELTA);

        // Slows the robot down as we go longer without a target. Hopefully allows the robot to "catch" the target again
        m_conditionedSpeed *= ((double) (resumeDrivingTickThreshold - ticksWithoutTarget) / resumeDrivingTickThreshold);

        m_conditionedSpeed = Utl.clip(m_conditionedSpeed, 0.0, maxSpeed);


        // ------- Calculate Direction -------
        m_conditionedDirection.atan2(calcY(), -calcX());

        // Add heading offset
        m_conditionedDirection.add(heading);


        // ------- Calculate Rotation --------
        m_conditionedRotate = new AngleD(heading).subtract(new AngleD(m_navx.getHeadingInfo().heading))
                .getRadians() * A05Constants.getDriveOrientationkp();


        // Update lasts
        m_lastConditionedDirection = m_conditionedDirection;
        m_lastConditionedSpeed = m_conditionedSpeed;
        m_lastConditionedRotate = m_conditionedRotate;

        driveSubsystem.swerveDrive(m_conditionedDirection, m_conditionedSpeed, m_conditionedRotate);

        if(Math.abs(camera.getXFromLastTarget() - xPosition) < inZoneThreshold && Math.abs(camera.getYFromLastTarget() - yPosition) < inZoneThreshold) {
            ticksInZoneCounter++;
            if(ticksInZoneCounter > ticksInZone) {
                isFinished = true;
            }
        }

        SmartDashboard.putNumber("cond. speed", m_conditionedSpeed);
        SmartDashboard.putNumber("cond. dir", m_conditionedDirection.getDegrees());
        SmartDashboard.putNumber("ticksInZone", ticksInZoneCounter);
        SmartDashboard.putNumber("noTarg", ticksWithoutTarget);
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the swerve
        driveSubsystem.swerveDrive(AngleConstantD.ZERO, 0.0, 0.0);
    }



    /**
     * Calculates the X speed between -1 and 1
     * @return x speed
     */
    private double calcX() {
        double center = (X_MAX + X_MIN) / 2.0;
        double scale = (X_MAX - X_MIN) / 2.0;
        return Utl.clip((camera.getXFromLastTarget() - center) / scale  -  (xPosition - center) / scale , -1.0, 1.0);
    }

    /**
     * Calculates the Y speed between -1 and 1
     * @return y speed
     */
    private double calcY() {
        double center = (Y_MAX + Y_MIN) / 2.0;
        double scale = (Y_MAX - Y_MIN) / 2.0;
        return Utl.clip((camera.getYFromLastTarget() - center) / scale  -  (yPosition - center) / scale, -1.0, 1.0);
    }
}
