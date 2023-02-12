package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;


public class ArmStickCommand extends CommandBase {

    private final ArmSubsystem m_armSubsystem = ArmSubsystem.getInstance();
    private final XboxController xbox;
    private final double DEADBAND = 0.05;
    private boolean pivotWasSpinning;
    private boolean extWasSpinning;

    public ArmStickCommand(XboxController xbox) {
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
        double xboxRight = -xbox.getRightY();
        double xboxLeft = -xbox.getLeftY();

        /*
        if (xboxRight > -DEADBAND && xboxRight < DEADBAND && extWasSpinning) {
            m_armSubsystem.setExtensionPosition(m_armSubsystem.getExtensionPosition());
            extWasSpinning = false;
        } else if (!(xboxRight > -DEADBAND && xboxRight < DEADBAND)){
            m_armSubsystem.setExtensionPower(xboxRight);
            extWasSpinning = true;
        }
        */

        if (xboxLeft > -DEADBAND && xboxLeft < DEADBAND && pivotWasSpinning) {
            m_armSubsystem.setPivotPosition(m_armSubsystem.getPivotPosition());
            pivotWasSpinning = false;
            m_armSubsystem.setExtensionPosition(m_armSubsystem.pivotToExtension());
        } else if(!(xboxLeft > -DEADBAND && xboxLeft < DEADBAND)){
            m_armSubsystem.setPivotPower(xboxLeft);
            pivotWasSpinning = true;
            m_armSubsystem.setExtensionPosition(m_armSubsystem.pivotToExtension());
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}
