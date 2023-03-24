package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.PhotonVisionSubsystem;
import org.a05annex.frc.commands.AbsoluteSmartTranslateCommand;
import org.a05annex.frc.subsystems.DriveSubsystem;
import org.a05annex.util.AngleD;


public class TranslateAprilTagCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();

    // Stores the current translate command
    private AbsoluteSmartTranslateCommand translate = null;

    private boolean isFinished;

    // Pointer to camera
    private final PhotonVisionSubsystem.Camera camera = Constants.DRIVE_CAMERA;

    private final double tolerance = 0.0254;

    private final double xPosition = 1.0;
    private final double yPosition = 0.0;

    // Constants of the drive
    private final double speed = 1.0;
    private final double accel = 4000.0;

    private final int latencyCycles = 5;
    private int latencyCycleCounter;

    private long startTime;

    public TranslateAprilTagCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        camera.updateLastFrameAndTarget();
        latencyCycleCounter = 0;
        startTime = System.currentTimeMillis();
        isFinished = false;

        driveSubsystem.swerveDrive(AngleD.ZERO, 0.0, 0.0);
    }

    @Override
    public void execute() {
        if (latencyCycleCounter < latencyCycles) {
            latencyCycleCounter++;
            return;
        }

        camera.updateLastFrameAndTarget();
        if(translate == null && camera.doLastFrameAndTargetMatch()) {
            if(Math.abs(camera.getXFromLastTarget() - xPosition) < tolerance && Math.abs(camera.getYFromLastTarget() - yPosition) < tolerance) {
                isFinished = true;
                return;
            }

            translate = new AbsoluteSmartTranslateCommand(camera.getXFromLastTarget() - xPosition, camera.getYFromLastTarget() - yPosition, speed, accel, false);
            translate.initialize();
            return;
        }
        if(translate == null) {
            return;
        }

        if(translate.isFinished()) {
            latencyCycleCounter = 0;
            if(Math.abs(camera.getXFromLastTarget() - xPosition) < tolerance && Math.abs(camera.getYFromLastTarget() - yPosition) < tolerance) {
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
        SmartDashboard.putNumber("total time", System.currentTimeMillis() - startTime);
    }
}
