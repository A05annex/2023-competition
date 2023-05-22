package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CollectorSubsystem;


public class SubstationConeCommand extends CommandBase {
    private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    private final CollectorSubsystem collectorSubsystem = CollectorSubsystem.getInstance();

    private ArmSubsystem.ArmPositions currentPosition = ArmSubsystem.ArmPositions.SUBSTATION_CONE_START;

    private int ticksAtPosition;

    public SubstationConeCommand() {
        addRequirements(this.armSubsystem, this.collectorSubsystem);
    }

    @Override
    public void initialize() {
        currentPosition = ArmSubsystem.ArmPositions.SUBSTATION_CONE_START;
        currentPosition.goTo();
        collectorSubsystem.spin();
        ticksAtPosition = 0;
    }

    @Override
    public void execute() {
        if(currentPosition.isInPosition()) {
            ticksAtPosition++;
            if(ticksAtPosition >= 0) {
                currentPosition = ArmSubsystem.ArmPositions.SUBSTATION_CONE_END;
                currentPosition.goTo();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return ArmSubsystem.ArmPositions.SUBSTATION_CONE_END.isInPosition();
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            new CollectorTimedIntakeCommand(100).schedule();
        }
    }
}
