package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CollectorSubsystem;


public class GroundPickupCommand extends CommandBase {
    private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    private final CollectorSubsystem collectorSubsystem = CollectorSubsystem.getInstance();

    public GroundPickupCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.armSubsystem, this.collectorSubsystem);
    }

    @Override
    public void initialize() {
        ArmSubsystem.ArmPositions.GROUND.goTo();
        collectorSubsystem.spin();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        ArmSubsystem.ArmPositions.RETRACTED.goTo();
        collectorSubsystem.stop();
    }
}
