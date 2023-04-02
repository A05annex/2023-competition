package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;


public class AutoStartingPlaceCommand extends CommandBase {
    private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();

    private final ArmSubsystem.ArmPositions position = ArmSubsystem.ArmPositions.CUBE_HIGH;

    private int ticks;

    public AutoStartingPlaceCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.armSubsystem);
    }

    @Override
    public void initialize() {
        position.goTo();
        ticks = -1;
    }

    @Override
    public void execute() {
        if(position.isInPosition()) {
            ticks++;
            if(ticks == 0) {
                new CollectorEjectCommand().schedule();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return position.isInPosition() && ticks >=5;
    }

    @Override
    public void end(boolean interrupted) {
        ArmSubsystem.ArmPositions.RETRACTED.goTo();
        //armSubsystem.setPivotPosition(-15.0);
    }
}
