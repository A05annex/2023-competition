package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;


public class AutoStartingPlaceCommand extends CommandBase {
    private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    private final ClawSubsystem clawSubsystem = ClawSubsystem.getInstance();

    private final double PIVOT = 18.0;
    private final double EXTENSION = 56.0;
    private final double DEADBAND = 0.5;
    private boolean isFinished = false;

    private int ticks;

    public AutoStartingPlaceCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.armSubsystem, this.clawSubsystem);
        ticks = -1;
        isFinished = false;
    }

    @Override
    public void initialize() {
        clawSubsystem.close();
        armSubsystem.setPivotPosition(PIVOT);
        armSubsystem.setExtensionPosition(EXTENSION);
        ticks = -1;
        isFinished = false;
    }

    @Override
    public void execute() {
        if(Math.abs(armSubsystem.getPivotPosition() - PIVOT) < DEADBAND &&
                Math.abs(armSubsystem.getExtensionPosition() - EXTENSION) < DEADBAND && ticks == -1)
        {
            ticks = 0;
        }
        if (ticks == 0) {
           clawSubsystem.open();
           ticks++;
        } else if (ticks == 5) {
            clawSubsystem.off();
            ticks++;
        } else if(ticks >= 75) {
            isFinished = true;
        } else if (ticks > 0) {
            ticks++;
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        ArmSubsystem.ArmPositions.RETRACTED.goTo();
    }
}
