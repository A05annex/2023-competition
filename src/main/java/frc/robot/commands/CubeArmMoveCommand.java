package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CollectorSubsystem;


public class CubeArmMoveCommand extends CommandBase {
    private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    private final CollectorSubsystem collectorSubsystem = CollectorSubsystem.getInstance();

    private final XboxController altXbox;
    private ArmSubsystem.ArmPositions position;


    public CubeArmMoveCommand(XboxController altXbox) {
        this.altXbox = altXbox;

        addRequirements(this.armSubsystem, this.collectorSubsystem);
    }

    @Override
    public void initialize() {
        if (altXbox.getPOV() == 0) {
            position = ArmSubsystem.ArmPositions.CUBE_HIGH;
        } else if (altXbox.getPOV() == -1) {
            position = ArmSubsystem.ArmPositions.CUBE_MEDIUM;
        } else if (altXbox.getPOV() == 180) {
            position = ArmSubsystem.ArmPositions.HYBRID;
        }

        position.goTo();
    }

    @Override
    public void execute() {
        if(Math.abs(position.getPivot() - armSubsystem.getPivotPosition()) < 2.0 && position == ArmSubsystem.ArmPositions.CUBE_MEDIUM) {
            new CollectorEjectCommand().schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return position.isInPosition();
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            new CollectorEjectCommand().schedule();
        }
    }
}
