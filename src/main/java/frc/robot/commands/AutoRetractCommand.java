package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CollectorSubsystem;


public class AutoRetractCommand extends CommandBase {
    private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    private final CollectorSubsystem collectorSubsystem = CollectorSubsystem.getInstance();

    public AutoRetractCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.armSubsystem, this.collectorSubsystem);
    }

    @Override
    public void initialize() {
        ArmSubsystem.ArmPositions.RETRACTED.goTo();
        collectorSubsystem.stop();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return ArmSubsystem.ArmPositions.RETRACTED.isInPosition();
    }

    @Override
    public void end(boolean interrupted) {

    }
}
