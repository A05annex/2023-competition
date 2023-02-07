package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.a05annex.frc.A05Constants;
import org.a05annex.util.Utl;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

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

    public PhotonPipelineResult lastTargetFrame = null;
    
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

    /**
     * Get the pipeline of requested camera
     * @param camera Camera you would like to get the pipeline of
     * @return pipeline of the camera passed in
     */
    public int getPipeline(PhotonCamera camera) {
        return camera.getPipelineIndex();
    }

    /**
     * Sets the pipeline for a camera
     * @param camera camera you would like to set the pipeline of
     * @param pipeline pipeline you want to set. int or pipeline from enum
     */
    public void setPipeline(PhotonCamera camera, PIPELINES pipeline) {
        camera.setPipelineIndex(pipeline.index);
    }

     // enum to store pipelines by name
    public enum PIPELINES {
        CUBE(0, "Cube"),
        CONE(1, "Cone"),
        APRILTAGS(2, "AprilTag");

        public final int index;
        public final String name;

        PIPELINES(int index, String name) {
            this.index = index;
            this.name = name;
        }
    }

    /**
     * Gets the newest frame and updates it to lastTargetFrame if there is a target in the shot
     * @param camera camera for which you would like to update the last frame
     */
    public void updateLastTarget(PhotonCamera camera) {
        PhotonPipelineResult old = lastTargetFrame;
        lastTargetFrame = camera.getLatestResult();
        if(!lastTargetFrame.hasTargets()) {
            lastTargetFrame = old;
        }
    }

    /**
     * Smooths out yaw values to account for inconsistent object recognition using exponential moving average formula
     * @param frame Frame you would like to grab the target from and pass into the yaw average
     * @return Smoothed out yaw offset
     */
    public double getYawOffsetAverage(PhotonPipelineResult frame) {
        if(frame.hasTargets()) {
            yawOffsetAverage = yawOffsetAverageAlpha * frame.getBestTarget().getYaw() + (1 - yawOffsetAverageAlpha) * yawOffsetAverage;
        }
        return yawOffsetAverage;
    }

    /**
     * Like getYawOffsetAverage, updates the average, but centers and scales to be between -1.0 and 1.0
     * @param frame Frame you would like to grab the target from and pass into the yaw average
     * @param minValue minimum expected value of what the target yaw will be
     * @param maxValue maximum expected value of what the target yaw will be
     * @return returns yawOffsetAverage that is clipped, centered and scaled between -1.0 and 1.0
     */
    public double getScaledCenteredYawOffsetAverage(PhotonPipelineResult frame, double minValue, double maxValue) {
        double yaw = Utl.clip(frame.getBestTarget().getYaw(), minValue, maxValue);
        if(frame.hasTargets()) {
            yawOffsetAverage = yawOffsetAverageAlpha * yaw + (1 - yawOffsetAverageAlpha) * yawOffsetAverage;
        }
        double center = (maxValue + minValue) / 2;
        double scale = (maxValue - minValue) / 2;
        return (yawOffsetAverage - center) / scale;
    }


    /**
     * Smooths out Area values to account for inconsistent object recognition using exponential moving average formula
     * @param frame Frame you would like to grab the target from and pass into the yaw average
     * @return Smoothed out area offset
     */
    public double getAreaOffsetAverage(PhotonPipelineResult frame) {
        if(frame.hasTargets()){
            areaOffsetAverage = areaOffsetAverageAlpha * frame.getBestTarget().getArea() + (1 - areaOffsetAverageAlpha) * areaOffsetAverage;
        } else {
            areaOffsetAverage = 0.0;
        }
        return areaOffsetAverage;
    }

    /**
     * Like {@link #getAreaOffsetAverage(PhotonPipelineResult)}, updates the average, but centers and scales to be between -1.0 and 1.0
     * @param frame Frame you would like to grab the target from and pass into the area average
     * @param minValue minimum expected value of what the target area will be
     * @param maxValue maximum expected value of what the target area will be
     * @return returns areaOffsetAverage that is clipped, centered and scaled between -1.0 and 1.0
     */
    public double getScaledCenteredAreaOffsetAverage(PhotonPipelineResult frame, double minValue, double maxValue) {
        double area = Utl.clip(frame.getBestTarget().getArea(), minValue, maxValue);
        if(frame.hasTargets()) {
            areaOffsetAverage = areaOffsetAverageAlpha * area + (1 - areaOffsetAverageAlpha) * areaOffsetAverage;
        }
        double center = (maxValue + minValue) / 2;
        double scale = (maxValue - minValue) / 2;
        return (areaOffsetAverage - center) / scale;
    }

    public PIPELINES intToPipeline(int pipeline) {
        if(!(0 <= pipeline && pipeline <= 2)) {
            if(A05Constants.getPrintDebug()) {
                System.out.println("passed integer outside of 0-2 into inToPipeline()");
            }
            return null;
        } else if(pipeline == 0) {
            return PIPELINES.CUBE;
        } else if(pipeline == 1) {
            return  PIPELINES.CONE;
        } else {
            return PIPELINES.APRILTAGS;
        }
    }
}

