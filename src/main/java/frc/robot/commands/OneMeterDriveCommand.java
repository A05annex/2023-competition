package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PhotonVisionSubsystem;
import org.a05annex.frc.subsystems.DriveSubsystem;


public class OneMeterDriveCommand extends CommandBase {
    private final DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();

    public OneMeterDriveCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.driveSubsystem);
    }

    @Override
    public void initialize() {
        driveSubsystem.translate(1.0, 0.0);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
