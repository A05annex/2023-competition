package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.a05annex.frc.NavX;
import org.a05annex.frc.subsystems.DriveSubsystem;
import org.a05annex.util.AngleConstantD;


public class AutoBalanceCommand extends CommandBase {
    private final DriveSubsystem m_driveSubsystem = DriveSubsystem.getInstance();

    // you need to use roll as pitch and roll return the opposite values of what you expect
    private AngleConstantD m_pitch = NavX.getInstance().getNavInfo().roll;
    // track during how many cylces (20ms) the robot was balanced
    private int ticksBalanced = 0;

    private boolean isFinished = false;


    public AutoBalanceCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(m_driveSubsystem);
    }

    @Override
    public void initialize() {
        isFinished = false;
        ticksBalanced = 0;
        m_pitch = NavX.getInstance().getNavInfo().roll;
    }

    @Override
    public void execute() {
        m_pitch = NavX.getInstance().getNavInfo().roll; // set pitch again because getNavInfo does not auto update
        if (m_pitch.getDegrees() > 10) {
            m_driveSubsystem.swerveDrive(new AngleConstantD(AngleConstantD.ZERO), 0.15, 0.0);
            ticksBalanced = 0;
        } else if (m_pitch.getDegrees() < -10) {
            m_driveSubsystem.swerveDrive(new AngleConstantD(AngleConstantD.ZERO), -0.15, 0.0);
            ticksBalanced = 0;
        } else {
            ticksBalanced++;
        }

        SmartDashboard.putNumber("pitch", m_pitch.getDegrees());
        SmartDashboard.putNumber("ticks balanced", ticksBalanced);
        SmartDashboard.putBoolean("is finished", isFinished);
    }

    @Override
    public boolean isFinished() {
        if(ticksBalanced > 150) {
            isFinished = true;
        }
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {}
}
