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
    private final double pitchOffsetAverageAlpha = 0.8;
    
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
        setPipeline(Constants.CLAW_CAMERA, PIPELINES.CONE);
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
        if (camera == Constants.CLAW_CAMERA) {
            PIPELINES.clawCurrent = pipeline;
        }
    }

     // enum to store pipelines by name
    public enum PIPELINES {
        CUBE(2, "Cube"),
        CONE(1, "Cone"),
        APRILTAGS(0, "AprilTag");

        public static PIPELINES clawCurrent = CONE;

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
    public PhotonPipelineResult updateLastTarget(@NotNull PhotonCamera camera, PhotonPipelineResult lastTargetFrame) {
        PhotonPipelineResult old = lastTargetFrame;
        lastTargetFrame = camera.getLatestResult();
        if(!lastTargetFrame.hasTargets()) {
            lastTargetFrame = old;
        }
        return lastTargetFrame;
    }

    /**
     * Smooths out yaw values to account for inconsistent object recognition using exponential moving average formula
     * @param frame Frame you would like to grab the target from and pass into the yaw average
     */
    public void calcYawOffsetAverage(@NotNull PhotonPipelineResult frame) {
        if(frame.hasTargets()) {
            yawOffsetAverage = yawOffsetAverageAlpha * frame.getBestTarget().getYaw() + (1 - yawOffsetAverageAlpha) * yawOffsetAverage;
        }
    }

    /**
     * Smooths out yaw values to account for inconsistent object recognition using exponential moving average formula,
     * but clips all values between the min and max.
     * @param frame Frame you would like to grab the target from and pass into the yaw average
     * @param minValue minimum expected value of what the target yaw will be
     * @param maxValue maximum expected value of what the target yaw will be
     */
    public void calcYawOffsetAverage(@NotNull PhotonPipelineResult frame, double minValue, double maxValue) {
        double yaw = Utl.clip(frame.getBestTarget().getYaw(), minValue, maxValue);
        if(frame.hasTargets()) {
            yawOffsetAverage = yawOffsetAverageAlpha * yaw + (1.0 - yawOffsetAverageAlpha) * yawOffsetAverage;
        }
    }

    /**
     * Get the unaltered yawOffsetAverage (might be clipped because of calcYawOffsetAverage)
     * @return yawOffsetAverage
     */
    public double getYawOffsetAverage() {
        return yawOffsetAverage;
    }

    /**
     * Returns the yawOffsetAverage but scales and centers it to be a double between -1 and 1
     * @param minValue minimum expected value of what the target yaw will be
     * @param maxValue maximum expected value of what the target will be
     * @param goal yaw value to target (keep between min and max)
     * @return translates the yawOffsetAverage to be a number between -1 and 1
     */
    public double getYawOffsetAverage(double minValue, double maxValue, double goal) {
        double center = (maxValue + minValue) / 2.0;
        double scale = (maxValue - minValue) / 2.0;
        return Utl.clip((yawOffsetAverage - center) / scale  -  (goal - center) / scale , -1.0, 1.0);
    }

    /**
     * Smooths out area values to account for inconsistent object recognition using exponential moving average formula
     * @param frame Frame you would like to grab the target from and pass into the area average
     */
    public void calcAreaOffsetAverage(@NotNull PhotonPipelineResult frame) {
        if(frame.hasTargets()){
            areaOffsetAverage = areaOffsetAverageAlpha * frame.getBestTarget().getArea() + (1.0 - areaOffsetAverageAlpha) * areaOffsetAverage;
        }
    }

    /**
     * Smooths out area values to account for inconsistent object recognition using exponential moving average formula,
     * but clips all values between the min and max.
     * @param frame Frame you would like to grab the target from and pass into the area average
     * @param minValue minimum expected value of what the target yaw will be
     * @param maxValue maximum expected value of what the target yaw will be
     */
    public void calcAreaOffsetAverage(@NotNull PhotonPipelineResult frame, double minValue, double maxValue) {
        double area = Utl.clip(frame.getBestTarget().getArea(), minValue, maxValue);
        if(frame.hasTargets()) {
            areaOffsetAverage = areaOffsetAverageAlpha * area + (1.0 - areaOffsetAverageAlpha) * areaOffsetAverage;
        }
    }

    /**
     * Get the unaltered areaOffsetAverage (might be clipped because of calcAreaOffsetAverage)
     * @return areaOffsetAverage
     */
    public double getAreaOffsetAverage() {
        return areaOffsetAverage;
    }

    /**
     * Returns the areaOffsetAverage but scales and centers it to be a double between -1 and 1
     * @param minValue minimum expected value of what the target yaw will be
     * @param maxValue maximum expected value of what the target will be
     * @param goal area value to target (keep between min and max)
     * @return translates the areaOffsetAverage to be a number between -1 and 1
     */
    public double getAreaOffsetAverage(double minValue, double maxValue, double goal) {
        double center = (maxValue + minValue) / 2.0;
        double scale = (maxValue - minValue) / 2.0;
        return Utl.clip((areaOffsetAverage - center) / scale  -  (goal - center) / scale , -1.0, 1.0);
    }


    /**
     * Smooths out pitch values to account for inconsistent object recognition using exponential moving average formula
     * @param frame Frame you would like to grab the target from and pass into the pitch average
     */
    public void calcPitchOffsetAverage(@NotNull PhotonPipelineResult frame) {
        if(frame.hasTargets()){
            pitchOffsetAverage = pitchOffsetAverageAlpha * frame.getBestTarget().getArea() + (1.0 - pitchOffsetAverageAlpha) * pitchOffsetAverage;
        }
    }

    /**
     * Smooths out pitch values to account for inconsistent object recognition using exponential moving average formula,
     * but clips all values between the min and max.
     * @param frame Frame you would like to grab the target from and pass into the pitch average
     * @param minValue minimum expected value of what the target yaw will be
     * @param maxValue maximum expected value of what the target yaw will be
     */
    public void calcPitchOffsetAverage(PhotonPipelineResult frame, double minValue, double maxValue) {
        double area = Utl.clip(frame.getBestTarget().getArea(), minValue, maxValue);
        if(frame.hasTargets()) {
            pitchOffsetAverage = pitchOffsetAverageAlpha * area + (1.0 - pitchOffsetAverageAlpha) * pitchOffsetAverage;
        }
    }

    /**
     * Get the unaltered pitchOffsetAverage (might be clipped because of calcPitchOffsetAverage)
     * @return pitchOffsetAverage
     */
    public double getPitchOffsetAverage() {
        return pitchOffsetAverage;
    }

    /**
     * Returns the pitchOffsetAverage but scales and centers it to be a double between -1 and 1
     * @param minValue minimum expected value of what the target yaw will be
     * @param maxValue maximum expected value of what the target will be
     * @param goal pitch value to target (keep between min and max)
     * @return translates the pitchOffsetAverage to be a number between -1 and 1
     */
    public double getPitchOffsetAverage(double minValue, double maxValue, double goal) {
        double center = (maxValue + minValue) / 2.0;
        double scale = (maxValue - minValue) / 2.0;
        return Utl.clip((pitchOffsetAverage - center) / scale  -  (goal - center) / scale , -1.0, 1.0);
    }

    public PIPELINES intToPipeline(int pipeline) {
        if(!(0 <= pipeline && pipeline <= 2)) {
            if(A05Constants.getPrintDebug()) {
                System.out.println("passed integer outside of 0-2 into inToPipeline()");
            }
            return null;
        } else if(pipeline == 0) {
            return PIPELINES.APRILTAGS;
        } else if(pipeline == 1) {
            return  PIPELINES.CONE;
        } else {
            return PIPELINES.CUBE;
        }
    }
}

