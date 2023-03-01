package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;


public class SubstationArmMoveCommand extends CommandBase {
    private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    private final ClawSubsystem clawSubsystem = ClawSubsystem.getInstance();

    public SubstationArmMoveCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.armSubsystem, this.clawSubsystem);
    }

    @Override
    public void initialize() {
        ArmSubsystem.ArmPositions.SUBSTATION_CUBE.goTo();
        clawSubsystem.open();
    }

    @Override
    public void execute() {
        if (PhotonVisionSubsystem.PIPELINES.clawCurrent == PhotonVisionSubsystem.PIPELINES.CONE) {
            ArmSubsystem.ArmPositions.SUBSTATION_CONE.goTo();
        } else {
            ArmSubsystem.ArmPositions.SUBSTATION_CUBE.goTo();
        }
    }

    @Override
    public boolean isFinished() {
        return ArmSubsystem.ArmPositions.currentPosition.isInPosition();
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            clawSubsystem.close();
        }
    }
}
