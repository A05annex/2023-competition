package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import org.a05annex.frc.subsystems.DriveSubsystem;


public class CubePlaceCommand extends CommandBase {
    private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    private final ClawSubsystem clawSubsystem = ClawSubsystem.getInstance();
    private final DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();
    private final XboxController altXbox;
    private ArmSubsystem.ArmPositions position;


    public CubePlaceCommand(XboxController altXbox) {
        this.altXbox = altXbox;
        addRequirements(this.armSubsystem, this.clawSubsystem, this.driveSubsystem);
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

    }
}
