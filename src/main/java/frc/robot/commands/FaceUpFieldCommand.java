package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.commands.A05DriveCommand;
import org.a05annex.frc.subsystems.DriveSubsystem;
import org.a05annex.util.AngleD;
import org.a05annex.util.Utl;


public class FaceUpFieldCommand extends A05DriveCommand {
    private final DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();

    public FaceUpFieldCommand(XboxController xbox, A05Constants.DriverSettings driver) {
        super(xbox, driver);
        addRequirements(this.driveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        conditionStick();
        m_navx.setExpectedHeading(m_navx.getHeadingInfo().getClosestUpField());
        m_conditionedRotate = new AngleD(m_navx.getHeadingInfo().expectedHeading).subtract(new AngleD(m_navx.getHeadingInfo().heading))
                .getRadians() * A05Constants.getDriveOrientationkp();
        m_conditionedRotate = Utl.clip(m_conditionedRotate, -0.5, 0.5);
        driveSubsystem.swerveDrive(m_conditionedDirection, m_conditionedSpeed, m_conditionedRotate);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
