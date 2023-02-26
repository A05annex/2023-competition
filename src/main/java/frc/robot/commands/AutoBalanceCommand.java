package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.a05annex.frc.NavX;
import org.a05annex.frc.subsystems.DriveSubsystem;
import org.a05annex.util.AngleConstantD;
import org.a05annex.util.AngleD;

/**
 * AutoBalanceCommand moves robot forward or backward based on robot tilt
 */
public class AutoBalanceCommand extends CommandBase {
    private final DriveSubsystem m_driveSubsystem = DriveSubsystem.getInstance();
    private final NavX m_navX = NavX.getInstance();
    // you need to use roll as pitch and roll return the opposite values of what you expect
    private AngleConstantD m_pitch = m_navX.getNavInfo().roll;

    // how fast should the robot drive (0.0 - 1.0) when trying to balance
    private final double m_speed = 0.15;
    private final double m_angle = 10.0;

    // track during how many cycles (20ms) the robot was balanced
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

        // set expected heading to either 0 or 180, whichever is closer to current heading
        if (m_navX.getHeading().isGreaterThan(AngleConstantD.DEG_NEG_90) && m_navX.getHeading().isLessThanOrEqual(AngleConstantD.DEG_90)) {
            m_navX.setExpectedHeading(new AngleD().setDegrees(0.0));
        }
        else {
            m_navX.setExpectedHeading(new AngleD().setDegrees(180.0));
        }
    }

    @Override
    public void execute() {
        m_pitch = NavX.getInstance().getNavInfo().roll; // set pitch again because getNavInfo does not auto update
        if (m_pitch.getDegrees() > m_angle) {
            // drive backward and reset ticks balanced when tipped forward
            m_driveSubsystem.swerveDrive(AngleConstantD.ZERO, m_speed, 0.0);
            ticksBalanced = 0;
        } else if (m_pitch.getDegrees() < -m_angle) {
            // drive forward and reset ticks balanced when tipped backward
            m_driveSubsystem.swerveDrive(AngleConstantD.ZERO, -m_speed, 0.0);
            ticksBalanced = 0;
        } else {
            // Stop robot and increment ticks balanced while not at full tilt
            m_driveSubsystem.swerveDrive(AngleConstantD.ZERO, 0.0, 0.0);
            ticksBalanced++;
        }

        SmartDashboard.putNumber("pitch", m_pitch.getDegrees());
        SmartDashboard.putNumber("ticks balanced", ticksBalanced);
        SmartDashboard.putBoolean("is finished", isFinished);
    }

    @Override
    public boolean isFinished() {
        // Is ticks balanced is greater than 150? ends the command if it is. 1 tick = 20ms. 20ms * 150 = 3 seconds
        if(ticksBalanced > 150) {
            isFinished = true;
        }
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {}
}
