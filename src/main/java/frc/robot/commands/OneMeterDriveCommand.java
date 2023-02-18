package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PhotonVisionSubsystem;
import org.a05annex.frc.commands.AbsoluteTranslateCommand;
import org.a05annex.frc.subsystems.DriveSubsystem;


public class OneMeterDriveCommand extends CommandBase {
    private final DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();
    AbsoluteTranslateCommand absoluteTranslateCommand = new AbsoluteTranslateCommand(0.0, 1.0);

    public OneMeterDriveCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.driveSubsystem);
    }

    @Override
    public void initialize() {
        absoluteTranslateCommand.initialize();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return absoluteTranslateCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
    }
}
