package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.PhotonVisionSubsystem;
import org.a05annex.frc.commands.AbsoluteSmartTranslateCommand;
import org.a05annex.frc.subsystems.DriveSubsystem;


public class TranslateAprilTagCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();

    // How many times you want the translate to run
    private final int iterations = 3;

    // Stores how many translates have run
    private int translatesCompleted;

    // Stores the current translate command
    private AbsoluteSmartTranslateCommand translate = null;

    private boolean isFinished;

    // Pointer to camera
    private final PhotonVisionSubsystem.Camera camera = Constants.DRIVE_CAMERA;

    private final double xPosition = 1.0;
    private final double yPosition = 0.119;

    // Constants of the drive
    private final double[] speed = {0.5, 0.2, 0.2};
    private final double[] accel = {5000.0, 2000.0, 1000.0};

    public TranslateAprilTagCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        camera.updateLastFrameAndTarget();
        translatesCompleted = 0;
        isFinished = false;
        if(camera.doLastFrameAndTargetMatch()) {
            translate = new AbsoluteSmartTranslateCommand(camera.getXFromLastTarget() - xPosition, camera.getYFromLastTarget() - yPosition, speed[translatesCompleted], accel[translatesCompleted], false);
            translate.initialize();
        }
    }

    @Override
    public void execute() {
        camera.updateLastFrameAndTarget();
        if(translate == null && camera.doLastFrameAndTargetMatch()) {
            translate = new AbsoluteSmartTranslateCommand(camera.getXFromLastTarget() - xPosition, camera.getYFromLastTarget() - yPosition, speed[translatesCompleted], accel[translatesCompleted], false);
            translate.initialize();
            return;
        }
        if(translate == null) {
            return;
        }

        if(translate.isFinished()) {
            translatesCompleted++;
            if(translatesCompleted == iterations) {
                isFinished = true;
            }
            translate = null;
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
