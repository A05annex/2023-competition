package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CollectorSubsystem;


public class ConeArmMoveCommand extends CommandBase {
    private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    private final CollectorSubsystem collectorSubsystem = CollectorSubsystem.getInstance();

    private final XboxController altXbox;
    private ArmSubsystem.ArmPositions position;

    public ConeArmMoveCommand(XboxController altXbox) {
        this.altXbox = altXbox;

        addRequirements(this.armSubsystem, this.collectorSubsystem);
    }

    @Override
    public void initialize() {
        if (altXbox.getPOV() == 90 || altXbox.getPOV() == 270) {
            position = ArmSubsystem.ArmPositions.CONE_MEDIUM;
        } else if (altXbox.getPOV() == 135 || altXbox.getPOV() == 225) {
            position = ArmSubsystem.ArmPositions.CONE_HYBRID;
        }

        position.goTo();
    }

    @Override
    public void execute() {

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
