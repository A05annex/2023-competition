package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CollectorSubsystem;


public class CollectorTimedIntakeCommand extends CommandBase {
    private final CollectorSubsystem collectorSubsystem = CollectorSubsystem.getInstance();

    private final int endTicks;
    private int ticksElapsed;

    public CollectorTimedIntakeCommand(int ticks) {
        endTicks = ticks;

        addRequirements(this.collectorSubsystem);
    }

    /**
     * Constructor that will default set intake ticks to 250 instead of taking in a number
     */
    public CollectorTimedIntakeCommand() {
        endTicks = 250;

        addRequirements(this.collectorSubsystem);
    }

    @Override
    public void initialize() {
        collectorSubsystem.spin();
    }

    @Override
    public void execute() {
        ticksElapsed++;
    }

    @Override
    public boolean isFinished() {
        return ticksElapsed >= endTicks;
    }

    @Override
    public void end(boolean interrupted) {
        collectorSubsystem.stop();
    }
}
