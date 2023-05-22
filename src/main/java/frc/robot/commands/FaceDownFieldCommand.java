package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.SpeedCachedSwerve;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.commands.A05DriveCommand;
import org.a05annex.frc.subsystems.DriveSubsystem;
import org.a05annex.util.AngleD;
import org.a05annex.util.Utl;


public class FaceDownFieldCommand extends A05DriveCommand {
    public FaceDownFieldCommand(XboxController xbox, A05Constants.DriverSettings driver) {
        // NOTE: the super adds the drive subsystem requirement
        super(SpeedCachedSwerve.getInstance(), xbox, driver);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        conditionStick();
        m_navx.setExpectedHeading(m_navx.getHeadingInfo().getClosestDownField());
        m_conditionedRotate = new AngleD(m_navx.getHeadingInfo().expectedHeading).subtract(new AngleD(m_navx.getHeadingInfo().heading))
                .getRadians() * A05Constants.getDriveOrientationkp();
        m_conditionedRotate = Utl.clip(m_conditionedRotate, -0.5, 0.5);
        iSwerveDrive.swerveDrive(m_conditionedDirection, m_conditionedSpeed, m_conditionedRotate);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
