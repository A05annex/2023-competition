package frc.robot.subsystems;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.a05annex.frc.A05Constants;
import org.a05annex.util.Utl;
import org.jetbrains.annotations.NotNull;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

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

    public static class Camera {
        private final PhotonCamera camera;
        //private final double horizontalFOVSize;
        //private final double verticalFOVSize;

        private PhotonPipelineResult lastFrame = new PhotonPipelineResult();
        private PhotonTrackedTarget lastTarget = new PhotonTrackedTarget();
        private double lastTargetId = -1;
        private double lastTargetTime = 0.0;

        private boolean doLastFrameAndTargetMatch;

        private PIPELINE currentPipeline;

        /**
         * Class to store a photon camera with relevant methods
         * @param camera PhotonCamera object you want to work with
         * @param currentPipeline stores the current pipeline of the camera and sets to this pipeline if not already set
         * //@param horizontalFOVSize horizontal FOV of the camera at whatever resolution you use (found in calibration)
         * //@param verticalFOVSize vertical FOV of the camera at whatever resolution you use (found in calibration)
         */
        public Camera(PhotonCamera camera, PIPELINE currentPipeline) {
            this.camera = camera;
            this.currentPipeline = currentPipeline;
            camera.setPipelineIndex(this.currentPipeline.index);
            //this.horizontalFOVSize = horizontalFOVSize;
            //this.verticalFOVSize = verticalFOVSize;
        }

        /**
         * Updates {@link #lastFrame} with the newest frame and if the frame has a valid target, sets {@link #lastTarget} too. Stores if
         * lastFrame and lastTarget have the same data in {@link #doLastFrameAndTargetMatch}.
         */
        public void updateLastFrameAndTarget() {
            lastFrame = camera.getLatestResult();
            if(lastFrame.hasTargets()) {
                lastTarget = lastFrame.getBestTarget();
                lastTargetTime = lastFrame.getTimestampSeconds();
                if (lastTargetId == lastTarget.getFiducialId()) {
                    // Yup, same target, update time, and we are good to go - we now have seen the same target
                    // as the best target atr least twice in a row.
                    doLastFrameAndTargetMatch = true;
                    return;
                } else {
                    // A different target is now our new best target
                    lastTargetId = lastTarget.getFiducialId();
                }
            }
            doLastFrameAndTargetMatch = false;
        }

        /**
         * Getter of {@link #lastFrame}. lastFrame is only updated by {@link #updateLastFrameAndTarget()}
         * @return PhotonPipelineResult of the last stored frame
         */
        public PhotonPipelineResult getLastFrame() {
            return lastFrame;
        }

        /**
         * Getter of {@link #lastTarget}. lastTarget is only updated by {@link #updateLastFrameAndTarget()}
         * @return PhotonTrackedTarget from the last stored frame to have a valid target
         */
        public PhotonTrackedTarget getLastTarget() {
            return lastTarget;
        }

        /**
         * Get the FPGA timestamp (in seconds) for the last tracked target.
         * @return The FPGA timestamp (in seconds) for the last tracked target
         */
        public double getLastTargetTime() {
            return lastTargetTime;
        }

        /**
         * Does the best target for the last retrieved frame match the previously saved best target in Id.
         *
         * @return true if it matches, false otherwise.
         */
        public boolean doLastFrameAndTargetMatch() {
            return doLastFrameAndTargetMatch;
        }

        /**
         * Does the best target for the last retrieved frame match the specified target Id.
         *
         * @param targetId The Id of the target we are looking for.
         * @return {@code true} if it matches, {@link false} otherwise.
         */
        public boolean doLastFrameAndTargetMatch(int targetId) {
            return lastFrame.hasTargets() && (lastTargetId == targetId);
        }

        /**
         * Getter of the current pipeline of the camera
         * @return PIPELINE of the current camera pipeline
         */
        public PIPELINE getCurrentPipeline() {
            return currentPipeline;
        }

        /**
         * Sets the pipeline of the camera
         * @param pipeline pipeline you want to set
         */
        public void setPipeline(PIPELINE pipeline) {
            if(pipeline == currentPipeline) {
                return;
            }

            currentPipeline = pipeline;
            camera.setPipelineIndex(currentPipeline.index);
        }

        public PhotonCamera getCamera() {
            return camera;
        }


        /**
         * Getter of the X distance to the target stored in {@link #lastTarget}.
         * May not be updated data if there has not been a target in frame recently
         * @return the X distance in meters (forward) from the camera to the target stored in {@link #lastTarget}
         */
        public double getXFromLastTarget() {
            return lastTarget.getBestCameraToTarget().getX();
        }

        /**
         * Getter of the Y distance to the target stored in {@link #lastTarget}.
         * May not be updated data if there has not been a target in frame recently
         * @return the Y distance in meters (forward) from the camera to the target stored in {@link #lastTarget}
         */
        public double getYFromLastTarget() {
            return -lastTarget.getBestCameraToTarget().getY();
        }

        /**
         * Getter of the Z distance to the target stored in {@link #lastTarget}.
         * May not be updated data if there has not been a target in frame recently
         * @return the Z distance in meters (forward) from the camera to the target stored in {@link #lastTarget}
         */
        public double getZFromLastTarget() {
            return lastTarget.getBestCameraToTarget().getZ();
        }
    }

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
    public void setPipeline(@NotNull PhotonCamera camera, @NotNull PhotonVisionSubsystem.PIPELINE pipeline) {
        camera.setPipelineIndex(pipeline.index);
    }

     // enum to store pipelines by name
    public enum PIPELINE {
        APRILTAGS(0),
        CONE(1),
        CUBE(2);

        public static PIPELINE clawCurrent = CONE;

        public final int index;

        PIPELINE(int index) {
            this.index = index;
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

    public PIPELINE intToPipeline(int pipeline) {
        if(!(0 <= pipeline && pipeline <= 2)) {
            if(A05Constants.getPrintDebug()) {
                System.out.println("passed integer outside of 0-2 into inToPipeline()");
            }
            return null;
        } else if(pipeline == 0) {
            return PIPELINE.APRILTAGS;
        } else if(pipeline == 1) {
            return  PIPELINE.CONE;
        } else {
            return PIPELINE.CUBE;
        }
    }

    public void canDriveTrue() {
        SmartDashboard.putBoolean("Can Drive", true);
    }

    public void canDriveFalse() {
        SmartDashboard.putBoolean("Can Drive", false);
    }
}

