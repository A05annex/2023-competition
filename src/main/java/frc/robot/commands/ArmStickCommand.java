package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;


public class ArmStickCommand extends CommandBase {

    private final ArmSubsystem m_armSubsystem = ArmSubsystem.getInstance();
    private final XboxController xbox;
    private final double DEADBAND = 0.05;

    public ArmStickCommand(XboxController xbox) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(m_armSubsystem);

        this.xbox = xbox;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double xboxRight = -xbox.getRightY();
        double xboxLeft = -xbox.getLeftY();

        if (xboxRight > -DEADBAND && xboxRight < DEADBAND) {
            m_armSubsystem.setExtensionPosition(m_armSubsystem.getExtensionPosition());
        } else {
            m_armSubsystem.setExtensionPower(xboxRight);
        }

        if (xboxLeft > -DEADBAND && xboxLeft < DEADBAND) {
            m_armSubsystem.setPivotPosition(m_armSubsystem.getPivotPosition());
        } else {
            m_armSubsystem.setPivotPower(xboxLeft);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_armSubsystem.stopAllMotors();
    }
}
