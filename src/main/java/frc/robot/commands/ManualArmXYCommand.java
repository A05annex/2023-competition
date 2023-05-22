package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmGeometry;
import frc.robot.subsystems.ArmSubsystem;

import java.awt.geom.Point2D;


public class ManualArmXYCommand extends CommandBase {

    private final ArmSubsystem m_armSubsystem = ArmSubsystem.getInstance();
    private final XboxController xbox;
    private final double DEADBAND = 0.05;
    private boolean wasSpinning;
    private Point2D.Double lastPt = null;

    public ManualArmXYCommand(XboxController xbox) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(m_armSubsystem);

        this.xbox = xbox;
    }

    @Override
    public void initialize() {
        wasSpinning = false;
        lastPt = null;
    }

    @Override
    public void execute() {
        if(!m_armSubsystem.isManualControl()) {
            wasSpinning = false;
            lastPt = null;
            return;
        }

        double xboxRight = -xbox.getRightY();
        double xboxLeft = -xbox.getLeftY();
        if ((xboxRight > -DEADBAND) && (xboxRight < DEADBAND) &&
                (xboxLeft > -DEADBAND) && (xboxLeft < DEADBAND)) {
            // no movement this cycle
            if (wasSpinning) {
                m_armSubsystem.setExtensionPositionDelta(0.0);
                wasSpinning = false;
            }
            return;
        }

        // get the current X,Y;
        // add the deltas;
        // convert back to positions;
        // clip; and then set the clipped positions
        if (null == lastPt) {
            lastPt = ArmGeometry.getArmLocationFromPositions(
                m_armSubsystem.getPivotPosition(),m_armSubsystem.getExtensionPosition());
        }
        Point2D.Double nextPt = new Point2D.Double(lastPt.x + 3.0*xboxRight,lastPt.y + 3.0*xboxLeft);
        ArmGeometry.ArmPosition position = ArmGeometry.getArmPositionsFromLocation(
                nextPt.x, nextPt.y);
        boolean clipped = position.clipToValidInPlay();
        m_armSubsystem.setExtensionPosition(position.getExtensionPosition());
        m_armSubsystem.setPivotPosition(position.getPivotPosition());
        lastPt = clipped ? null : nextPt;
        wasSpinning = true;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
