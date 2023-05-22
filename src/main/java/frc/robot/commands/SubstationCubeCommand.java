package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CollectorSubsystem;


public class SubstationCubeCommand extends CommandBase {
    private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    private final CollectorSubsystem collectorSubsystem = CollectorSubsystem.getInstance();

    private final ArmSubsystem.ArmPositions currentPosition = ArmSubsystem.ArmPositions.SUBSTATION_CUBE;

    public SubstationCubeCommand() {
        addRequirements(this.armSubsystem, this.collectorSubsystem);
    }

    @Override
    public void initialize() {
        currentPosition.goTo();
        collectorSubsystem.spin();
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return currentPosition.isInPosition();
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            new CollectorTimedIntakeCommand(50).schedule();
        }
    }
}
