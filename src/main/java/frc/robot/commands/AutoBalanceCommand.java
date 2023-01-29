package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import org.a05annex.frc.NavX;
import org.a05annex.frc.subsystems.DriveSubsystem;
import org.a05annex.util.AngleConstantD;
import org.a05annex.util.AngleD;


public class AutoBalanceCommand extends CommandBase {
    private final NavX m_navX = NavX.getInstance();
    private NavX.NavInfo m_navInfo = m_navX.getNavInfo();
    private AngleConstantD m_pitch = m_navInfo.pitch;
    private final DriveSubsystem m_driveSubsystem = DriveSubsystem.getInstance();
    private int ticksBalanced = 0;

    private boolean isFinished = false;



    public AutoBalanceCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(m_driveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (m_pitch.getDegrees() > 10) {
            m_driveSubsystem.swerveDrive(new AngleConstantD(AngleConstantD.ZERO), 0.15, 0.0);
            ticksBalanced = 0;
        } else if (m_pitch.getDegrees() > 10) {
            m_driveSubsystem.swerveDrive(new AngleConstantD(AngleConstantD.ZERO), -0.15, 0.0);
            ticksBalanced = 0;
        } else {
            ticksBalanced++;
        }
        SmartDashboard.putNumber("ticks bal.", ticksBalanced);
        SmartDashboard.putNumber("pitch", m_pitch.getDegrees());
        SmartDashboard.putNumber("raw pitch", m_navInfo.rawPitch.getDegrees());
        SmartDashboard.putNumber("roll", m_navInfo.roll.getDegrees());
        SmartDashboard.putNumber("raw roll", m_navInfo.rawRoll.getDegrees());
    }

    @Override
    public boolean isFinished() {
        if(ticksBalanced > 150) {
            isFinished = true;
        }
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
