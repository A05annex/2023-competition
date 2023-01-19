package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVisionSubsystem extends SubsystemBase {

    // With eager singleton initialization, any static variables/fields used in the 
    // constructor must appear before the "INSTANCE" variable so that they are initialized 
    // before the constructor is called when the "INSTANCE" variable initializes.

    /**
     * The Singleton instance of this PhotonVisionSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static PhotonVisionSubsystem INSTANCE = new PhotonVisionSubsystem();

    private double yawOffsetAverage = 0.0;
    public double yawOffsetAverageAlpha = 0.9;

    private double areaOffsetAverage = 0.0;
    public double areaOffsetAverageAlpha = 0.9;

    public PhotonPipelineResult lastResult = null;
    /**
     * Returns the Singleton instance of this PhotonVisionSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code PhotonVisionSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static PhotonVisionSubsystem getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this PhotonVisionSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private PhotonVisionSubsystem() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
        setPipeline(Constants.DRIVE_CAMERA, PIPELINES.APRILTAGS);
    }

    public int getPipeline(PhotonCamera camera) {
        return camera.getPipelineIndex();
    }

    public void setPipeline(PhotonCamera camera, PIPELINES pipeline) {
        camera.setPipelineIndex(pipeline.index);
    }

    public enum PIPELINES {
        CUBE(0),
        CONE(1),
        APRILTAGS(2);

        private final int index;

        PIPELINES(int index) {
            this.index = index;
        }
    }

    public void updateLastTarget(PhotonCamera camera) {
        PhotonPipelineResult old = lastResult;
        lastResult = camera.getLatestResult();
        if(!lastResult.hasTargets()) {
            lastResult = old;
        }
    }

    public boolean hasTarget(PhotonPipelineResult result) {
        return result.hasTargets();
    }

    public double getYawOffsetAverage(PhotonPipelineResult result) {
        if(result.hasTargets()) {
            yawOffsetAverage = yawOffsetAverageAlpha * result.getBestTarget().getYaw() + (1 - yawOffsetAverageAlpha) * yawOffsetAverage;
        } else {
            yawOffsetAverage = 0.0;
        }
        return yawOffsetAverage;
    }

    public double getAreaOffsetAverage(PhotonPipelineResult result) {
        if(result.hasTargets()){
            areaOffsetAverage = areaOffsetAverageAlpha * result.getBestTarget().getArea() + (1 - areaOffsetAverageAlpha) * areaOffsetAverage;
        } else {
            areaOffsetAverage = 0.0;
        }
        return areaOffsetAverage;
    }
}

