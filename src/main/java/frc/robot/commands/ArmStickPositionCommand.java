package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmGeometry;
import frc.robot.subsystems.ArmSubsystem;

import java.awt.geom.Point2D;

/**
 * This is a command that uses the xbox controller left and right stick to control
 * the X (away from or towards the robot), and Y (the height above the floor) of the
 * end of the control arm.
 */
public class ArmStickPositionCommand extends CommandBase {
    private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    private final XboxController xbox;
    private final double DEADBAND = 0.05;

    private final double MAX_INCHES_PER_CYCLE = 1.0;
    Point2D.Double currentLocation =
            ArmGeometry.getArmLocationFromPositions(armSubsystem.getPivotPosition(),
                    armSubsystem.getExtensionPosition());
    Point2D.Double lastLocation = currentLocation;
    boolean lastLocationWasClipped = false;

    public ArmStickPositionCommand(XboxController xbox) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.armSubsystem);
        this.xbox = xbox;
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        // Save the current X,Y location. Note, this is probably the start position which is at
        // a stop for extension and a known pivot angle.
        currentLocation =
                ArmGeometry.getArmLocationFromPositions(armSubsystem.getPivotPosition(),
                        armSubsystem.getExtensionPosition());
        lastLocation = currentLocation;
        lastLocationWasClipped = false;
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled
     * (That is, it is called repeatedly until {@link #isFinished()} returns {@code true}).
     */
    @Override
    public void execute() {
        // get the dX and dY from the left and right stick
        double xboxRight = xbox.getRightY(); // right forward is down (-Y)
        double dY = (xboxRight > DEADBAND) ? ((xboxRight - DEADBAND) / (1.0 - DEADBAND)) :
                (xboxRight < DEADBAND) ? ((xboxRight + DEADBAND) / (1.0 - DEADBAND)) : 0.0;
        double xboxLeft = -xbox.getLeftY(); // left forward is out (+X)
        double dX = (xboxLeft > DEADBAND) ? ((xboxLeft - DEADBAND) / (1.0 - DEADBAND)) :
                (xboxLeft < DEADBAND) ? ((xboxLeft + DEADBAND) / (1.0 - DEADBAND)) : 0.0;

        double newX = lastLocation.x + dX;
        double newY = lastLocation.y + dY;
        ArmGeometry.ArmPositions newPositions = ArmGeometry.getArmPositionsFromLocation(newX, newY);
        boolean clipped = newPositions.clipToValidInPlay();
        if (!clipped || !lastLocationWasClipped) {
            // The new position is a valid position and was not clipped - move there.
            // OR - this is a clipped location on the edge of the valid position window..
            armSubsystem.setPivotPosition(newPositions.getPivotPosition());
            armSubsystem.setExtensionPosition(newPositions.getExtensionPosition());
            currentLocation =
                    ArmGeometry.getArmLocationFromPositions(armSubsystem.getPivotPosition(),
                            armSubsystem.getExtensionPosition());
            lastLocation = currentLocation;
        }
        lastLocationWasClipped = clipped;
    }

    /**
     * <p>
     * Returns whether this command has finished. Once a command finishes -- indicated by
     * this method returning true -- the scheduler will call its {@link #end(boolean)} method.
     * </p><p>
     * Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Hard coding this command to always
     * return true will result in the command executing once and finishing immediately. It is
     * recommended to use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand}
     * for such an operation.
     * </p>
     *
     * @return whether this command has finished.
     */
    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    /**
     * The action to take when the command ends. Called when either the command
     * finishes normally -- that is it is called when {@link #isFinished()} returns
     * true -- or when  it is interrupted/canceled. This is where you may want to
     * wrap up loose ends, like shutting off a motor that was being used in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {

    }
}
