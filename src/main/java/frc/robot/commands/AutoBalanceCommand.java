package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.NavX;
import org.a05annex.frc.subsystems.DriveSubsystem;
import org.a05annex.util.AngleConstantD;
import org.a05annex.util.AngleD;

/**
 * AutoBalanceCommand moves robot forward or backward based on robot tilt
 */
public class AutoBalanceCommand extends CommandBase {
    private final DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();
    private final NavX navX = NavX.getInstance();
    // you need to use roll as pitch and roll return the opposite values of what you expect
    private AngleConstantD m_pitch = navX.getNavInfo().roll;

    // how fast should the robot drive (0.0 - 1.0) when trying to balance
    private final double SPEED = 0.18;
    // At what degree angle should the robot stop moving. (platform max tilt is 15°)
    private final double ANGLE = 12.0;

    private double upField;

    private double speed = 0.0, rotation = 0.0;

    // track during how many cycles (20ms) the robot was balanced
    private int ticksBalanced = 0;

    private boolean isFinished = false;


    public AutoBalanceCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.driveSubsystem);
    }

    @Override
    public void initialize() {
        isFinished = false;
        ticksBalanced = 0;
        m_pitch = NavX.getInstance().getNavInfo().roll;
        upField = navX.getHeadingInfo().getClosestDownOrUpField() == navX.getHeadingInfo().getClosestDownField() ? -1.0 : 1.0;
    }

    @Override
    public void execute() {
        upField = navX.getHeadingInfo().getClosestDownOrUpField().equals(navX.getHeadingInfo().getClosestDownField()) ? -1.0 : 1.0;

        m_pitch = NavX.getInstance().getNavInfo().roll; // set pitch again because getNavInfo does not auto update
        if (m_pitch.getDegrees() < -ANGLE) {
            // drive backward and reset ticks balanced when tipped forward
            speed = -SPEED;
            ticksBalanced = 0;
        } else if (m_pitch.getDegrees() > ANGLE) {
            // drive forward and reset ticks balanced when tipped backward
            speed = SPEED;
            ticksBalanced = 0;
        } else {
            // Stop robot and increment ticks balanced while not at full tilt
            speed = 0.0;
            ticksBalanced++;
        }

        navX.setExpectedHeading(navX.getHeadingInfo().getClosestDownOrUpField());
        rotation = new AngleD(navX.getHeadingInfo().expectedHeading).subtract(new AngleD(navX.getHeadingInfo().heading))
                .getRadians() * A05Constants.getDriveOrientationkp();
        driveSubsystem.swerveDrive(AngleConstantD.ZERO, speed * upField, rotation);
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
