package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CollectorSubsystem;


public class CollectorEjectCommand extends CommandBase {
    private final CollectorSubsystem collectorSubsystem = CollectorSubsystem.getInstance();

    private final int runTicks = 35;
    private int ticksElapsed;

    public CollectorEjectCommand() {
        addRequirements(this.collectorSubsystem);
    }

    @Override
    public void initialize() {
        ticksElapsed = 0;
        collectorSubsystem.spinAtSpeed(-3000);
    }

    @Override
    public void execute() {
        ticksElapsed++;
    }

    @Override
    public boolean isFinished() {
        return ticksElapsed >= runTicks;
    }

    @Override
    public void end(boolean interrupted) {
        collectorSubsystem.stop();
    }
}
