package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.a05annex.frc.A05Constants;
import org.a05annex.util.Utl;
import org.jetbrains.annotations.NotNull;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonVisionSubsystem extends SubsystemBase {

    /**
     * The Singleton instance of this PhotonVisionSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static PhotonVisionSubsystem INSTANCE = new PhotonVisionSubsystem();

    private double yawOffsetAverage = 0.0;
    private final double yawOffsetAverageAlpha = 0.8;

    private double areaOffsetAverage = 0.0;
    private final double areaOffsetAverageAlpha = 0.8;

    private double pitchOffsetAverage = 0.0;
    private final double pitchOffsetAverageAlpha = 0.9;

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
        //setPipeline(Constants.DRIVE_CAMERA, PIPELINES.APRILTAGS);
    }

    /**
     * Get the pipeline of requested camera
     * @param camera Camera you would like to get the pipeline of
     * @return pipeline of the camera passed in
     */
    public int getPipeline(@NotNull PhotonCamera camera) {
        return camera.getPipelineIndex();
    }

    /**
     * Sets the pipeline for a camera
     * @param camera camera you would like to set the pipeline of
     * @param pipeline pipeline you want to set. int or pipeline from enum
     */
    public void setPipeline(@NotNull PhotonCamera camera, @NotNull PIPELINES pipeline) {
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
    public void updateLastTarget(@NotNull PhotonCamera camera) {
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
    public double calcYawOffsetAverage(@NotNull PhotonPipelineResult frame) {
        if(frame.hasTargets()) {
            yawOffsetAverage = yawOffsetAverageAlpha * frame.getBestTarget().getYaw() + (1 - yawOffsetAverageAlpha) * yawOffsetAverage;
        }
        return yawOffsetAverage;
    }

    /**
     * Smooths out yaw values to account for inconsistent object recognition using exponential moving average formula,
     * but centers and scales to be between -1.0 and 1.0
     * @param frame Frame you would like to grab the target from and pass into the yaw average
     * @param minValue minimum expected value of what the target yaw will be
     * @param maxValue maximum expected value of what the target yaw will be
     * @return returns yawOffsetAverage that is clipped, centered and scaled between -1.0 and 1.0
     */
    public double calcYawOffsetAverage(@NotNull PhotonPipelineResult frame, double minValue, double maxValue) {
        double yaw = Utl.clip(frame.getBestTarget().getYaw(), minValue, maxValue);
        if(frame.hasTargets()) {
            yawOffsetAverage = yawOffsetAverageAlpha * yaw + (1 - yawOffsetAverageAlpha) * yawOffsetAverage;
        }
        double center = (maxValue + minValue) / 2;
        double scale = (maxValue - minValue) / 2;
        return (yawOffsetAverage - center) / scale;
    }

    /**
     * Smooths out yaw values to account for inconsistent object recognition using exponential moving average formula,
     * but offsets the target, then centers and scales to be between -1.0 and 1.0
     * @param frame Frame you would like to grab the target from and pass into the yaw average
     * @param minValue minimum expected value of what the target yaw will be
     * @param maxValue maximum expected value of what the target yaw will be
     * @param offset offset from center you want (between min and max, not -1 and 1)
     * @return returns yawOffsetAverage that is clipped, centered and scaled between -1.0 and 1.0, but shifted
     */
    public double calcYawOffsetAverage(PhotonPipelineResult frame, double minValue, double maxValue, double offset) {
        double yaw = Utl.clip(frame.getBestTarget().getYaw(), minValue, maxValue);
        if(frame.hasTargets()) {
            yawOffsetAverage = yawOffsetAverageAlpha * yaw + (1 - yawOffsetAverageAlpha) * yawOffsetAverage;
        }
        double center = (maxValue + minValue) / 2;
        double scale = (maxValue - minValue) / 2;
        return Utl.clip((yawOffsetAverage - center - offset) / scale, minValue, maxValue);
    }

    public double getYawOffsetAverage() {
        return yawOffsetAverage;
    }

    public double getYawOffsetAverage(double minValue, double maxValue, double offset) {
        double center = (maxValue + minValue) / 2;
        double scale = (maxValue - minValue) / 2;
        return Utl.clip((yawOffsetAverage - center - offset) / scale, minValue, maxValue);
    }

    /**
     * Smooths out area values to account for inconsistent object recognition using exponential moving average formula
     * @param frame Frame you would like to grab the target from and pass into the yaw average
     * @return Smoothed out area offset
     */
    public double calcAreaOffsetAverage(@NotNull PhotonPipelineResult frame) {
        if(frame.hasTargets()){
            areaOffsetAverage = areaOffsetAverageAlpha * frame.getBestTarget().getArea() + (1 - areaOffsetAverageAlpha) * areaOffsetAverage;
        } else {
            areaOffsetAverage = 0.0;
        }
        return areaOffsetAverage;
    }

    /**
     * Smooths out area values to account for inconsistent object recognition using exponential moving average formula,
     * but centers and scales to be between -1.0 and 1.0
     * @param frame Frame you would like to grab the target from and pass into the area average
     * @param minValue minimum expected value of what the target area will be
     * @param maxValue maximum expected value of what the target area will be
     * @return returns areaOffsetAverage that is clipped, centered and scaled between -1.0 and 1.0
     */
    public double calcAreaOffsetAverage(@NotNull PhotonPipelineResult frame, double minValue, double maxValue) {
        double area = Utl.clip(frame.getBestTarget().getArea(), minValue, maxValue);
        if(frame.hasTargets()) {
            areaOffsetAverage = areaOffsetAverageAlpha * area + (1 - areaOffsetAverageAlpha) * areaOffsetAverage;
        }
        double center = (maxValue + minValue) / 2;
        double scale = (maxValue - minValue) / 2;
        return (areaOffsetAverage - center) / scale;
    }

    /**
     * Smooths out area values to account for inconsistent object recognition using exponential moving average formula,
     * but offsets the target, then centers and scales to be between -1.0 and 1.0
     * @param frame Frame you would like to grab the target from and pass into the yaw average
     * @param minValue minimum expected value of what the target yaw will be
     * @param maxValue maximum expected value of what the target yaw will be
     * @param offset offset from center you want (between min and max, not -1 and 1)
     * @return returns yawOffsetAverage that is clipped, centered and scaled between -1.0 and 1.0, but shifted
     */
    public double calcAreaOffsetAverage(@NotNull PhotonPipelineResult frame, double minValue, double maxValue, double offset) {
        double yaw = Utl.clip(frame.getBestTarget().getYaw(), minValue, maxValue);
        if(frame.hasTargets()) {
            areaOffsetAverage = areaOffsetAverageAlpha * yaw + (1 - areaOffsetAverageAlpha) * areaOffsetAverage;
        }
        double center = (maxValue + minValue) / 2;
        double scale = (maxValue - minValue) / 2;
        return Utl.clip((areaOffsetAverage - center - offset) / scale, minValue, maxValue);
    }

    public double getAreaOffsetAverage() {
        return areaOffsetAverage;
    }

    public double getAreaOffsetAverage(double minValue, double maxValue, double offset) {
        double center = (maxValue + minValue) / 2;
        double scale = (maxValue - minValue) / 2;
        return Utl.clip((areaOffsetAverage - center - offset) / scale, minValue, maxValue);
    }

    /**
     * Smooths out pitch values to account for inconsistent object recognition using exponential moving average formula
     * @param frame Frame you would like to grab the target from and pass into the yaw average
     * @return Smoothed out area offset
     */
    public double calcPitchOffsetAverage(@NotNull PhotonPipelineResult frame) {
        if(frame.hasTargets()){
            pitchOffsetAverage = pitchOffsetAverageAlpha * frame.getBestTarget().getArea() + (1 - pitchOffsetAverageAlpha) * pitchOffsetAverage;
        } else {
            pitchOffsetAverage = 0.0;
        }
        return pitchOffsetAverage;
    }

    /**
     * Smooths out pitch values to account for inconsistent object recognition using exponential moving average formula,
     * but centers and scales to be between -1.0 and 1.0
     * @param frame Frame you would like to grab the target from and pass into the area average
     * @param minValue minimum expected value of what the target area will be
     * @param maxValue maximum expected value of what the target area will be
     * @return returns areaOffsetAverage that is clipped, centered and scaled between -1.0 and 1.0
     */
    public double calcPitchOffsetAverage(PhotonPipelineResult frame, double minValue, double maxValue) {
        double area = Utl.clip(frame.getBestTarget().getArea(), minValue, maxValue);
        if(frame.hasTargets()) {
            pitchOffsetAverage = pitchOffsetAverageAlpha * area + (1 - pitchOffsetAverageAlpha) * pitchOffsetAverage;
        }
        double center = (maxValue + minValue) / 2;
        double scale = (maxValue - minValue) / 2;
        return (pitchOffsetAverage - center) / scale;
    }

    /**
     * Smooths out pitch values to account for inconsistent object recognition using exponential moving average formula,
     * but offsets the target, then centers and scales to be between -1.0 and 1.0
     * @param frame Frame you would like to grab the target from and pass into the yaw average
     * @param minValue minimum expected value of what the target yaw will be
     * @param maxValue maximum expected value of what the target yaw will be
     * @param offset offset from center you want (between min and max, not -1 and 1)
     * @return returns yawOffsetAverage that is clipped, centered and scaled between -1.0 and 1.0, but shifted
     */
    public double calcPitchOffsetAverage(PhotonPipelineResult frame, double minValue, double maxValue, double offset) {
        double yaw = Utl.clip(frame.getBestTarget().getYaw(), minValue, maxValue);
        if(frame.hasTargets()) {
            pitchOffsetAverage = pitchOffsetAverageAlpha * yaw + (1 - pitchOffsetAverageAlpha) * pitchOffsetAverage;
        }
        double center = (maxValue + minValue) / 2;
        double scale = (maxValue - minValue) / 2;
        return Utl.clip((pitchOffsetAverage - center - offset) / scale, minValue, maxValue);
    }

    public double getPitchOffsetAverage() {
        return pitchOffsetAverage;
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

