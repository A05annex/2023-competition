package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;


public class ManualArmCommand extends CommandBase {

    private final ArmSubsystem m_armSubsystem = ArmSubsystem.getInstance();
    private final XboxController xbox;
    private final double DEADBAND = 0.05;
    private boolean pivotWasSpinning;
    private boolean extWasSpinning;

    public ManualArmCommand(XboxController xbox) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(m_armSubsystem);

        this.xbox = xbox;
    }

    @Override
    public void initialize() {
        pivotWasSpinning = false;
        extWasSpinning = false;
    }

    @Override
    public void execute() {
        if(!m_armSubsystem.isManualControl()) {
            pivotWasSpinning = false;
            extWasSpinning = false;
            return;
        }

        double xboxRight = -xbox.getRightY();
        double xboxLeft = -xbox.getLeftY();


        if (xboxRight > -DEADBAND && xboxRight < DEADBAND) {
            if (extWasSpinning) {
                m_armSubsystem.setExtensionPositionDelta(0.0);
                extWasSpinning = false;
            }
        } else {
            m_armSubsystem.setExtensionPositionDelta(xboxRight * 8.0);
            extWasSpinning = true;
        }


        if (xboxLeft > -DEADBAND && xboxLeft < DEADBAND) {
            if (pivotWasSpinning) {
                m_armSubsystem.setPivotPositionDelta(0.0);
                pivotWasSpinning = false;
            }
        } else {
            m_armSubsystem.setPivotPositionDelta(xboxLeft * 8.0);
            pivotWasSpinning = true;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
