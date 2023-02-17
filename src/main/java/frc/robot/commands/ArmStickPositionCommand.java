package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmGeometry;
import frc.robot.subsystems.ArmSubsystem;

import java.awt.geom.Point2D;


public class ArmStickPositionCommand extends CommandBase {
    private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    private final XboxController xbox;
    private final double DEADBAND = 0.05;

    private final double INCHES_PER_CYCLE = 1.0;
    Point2D.Double currentPosition =
            ArmGeometry.getArmLocationFromPositions(armSubsystem.getPivotPosition(),
                    armSubsystem.getExtensionPosition());
    Point2D.Double lastPosition = currentPosition;
    boolean lastPositionWasClipped = false;

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
        currentPosition =
                ArmGeometry.getArmLocationFromPositions(armSubsystem.getPivotPosition(),
                        armSubsystem.getExtensionPosition());
        lastPosition = currentPosition;
        lastPositionWasClipped = false;

    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        // get the dX and dY from the left and right stick
        double xboxRight = xbox.getRightY(); // right forward is down
        double dY = (xboxRight > DEADBAND) ? ((xboxRight - DEADBAND) / (1.0 - DEADBAND)) :
                (xboxRight < DEADBAND) ? ((xboxRight + DEADBAND) / (1.0 - DEADBAND)) : 0.0;
        double xboxLeft = -xbox.getLeftY(); // left forward is out
        double dX = 0.0;

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
