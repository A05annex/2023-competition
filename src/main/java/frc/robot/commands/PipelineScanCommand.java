package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PhotonVisionSubsystem;
import org.photonvision.PhotonCamera;


public class PipelineScanCommand extends CommandBase {

    private final PhotonVisionSubsystem m_photonSubsystem = PhotonVisionSubsystem.getInstance();

    private final PhotonCamera camera;

    // Stores the current pipeline
    private PhotonVisionSubsystem.PIPELINES currentPipeline;

    // Counter to keep track of how many ticks it's been since there has been no target
    private int ticksLost;

    // Used to make sure the camera has time to switch between pipelines
    private Integer coolDown;

    public PipelineScanCommand(PhotonCamera camera) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        this.camera = camera;
        SmartDashboard.putString("last target on", "init");
        addRequirements();

        coolDown = 0;
    }

    @Override
    public void initialize() {
        currentPipeline = m_photonSubsystem.intToPipeline(m_photonSubsystem.getPipeline(camera));
        ticksLost = 0;
        //coolDown = 0;
    }

    @Override
    public void execute() {
        // Is there a target?
        // Yes. great, send telemetry and reset the counter
        SmartDashboard.putNumber("cool down", coolDown);
        SmartDashboard.putNumber("ticks lost", ticksLost);

        if(coolDown <= 0) {
            if (camera.getLatestResult().hasTargets()) {
                ticksLost = 0;
                SmartDashboard.putString("last target on", currentPipeline.name);
            }
            // No Target.
            // Have you had a target recently? (last 5 ticks. 20ms * 5 = 0.1 seconds)
            // Yes. wait it out but start counting how long there hasn't been a target for.
            else if (!camera.getLatestResult().hasTargets() && ticksLost < 5) {
                ticksLost++;
            }
            // No Target recently. try the other pipeline.
            else {
                if (currentPipeline == PhotonVisionSubsystem.PIPELINES.CONE) {
                    currentPipeline = PhotonVisionSubsystem.PIPELINES.CUBE;
                } else {
                    currentPipeline = PhotonVisionSubsystem.PIPELINES.CONE;
                }
                m_photonSubsystem.setPipeline(camera, currentPipeline);

                SmartDashboard.putString("last target on", "none");

                coolDown = 15; // 15 ticks. 15 * 0.02 = 0.3 seconds
            }
        } else {
            coolDown--;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
