package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.subsystems.SparkNeo;
import org.a05annex.util.Utl;

public class ArmSubsystem extends SubsystemBase {
    private boolean enableInit = false;
    private boolean manualControl = false;

    // Declaring everything for the forward support pivot motor
    private final SparkNeo forwardPivot;
    private final SparkNeo backwardPivot;

    //Array of positions. [starting position, min position, max position]
    private final double[] pivotPositions = {0, -40, 37.3};

    private final double pivotSmKp = 0.00005, pivotSmKi = 0.000, pivotSmKiZone = 0.0, pivotSmKff = 0.000156,
            pivotSmMaxRPM = 3000.0, pivotSmMaxRPMs = 3000.0, pivotSmMinRPM = 0.0, pivotSmError = 0.1;
    private final double pivotPosKp = 0.22, pivotPosKi = 0.0, pivotPosKiZone = 0.0, pivotPosKff = 0.0;
    private final double pivotTicksPerRotation = 30.309 * 4; //Reading from 0 to 90 degrees * 4 = full rotation
    private double lastPivotSetReference = 0.0;

    // Declaring everything for the extension motor
    private final SparkNeo extend;

    // Array of positions. [starting position, min position, max position]
    private final double[] extensionPositions = {0.0, 0.125, 150.73};
    private final double extensionKP = 0.00005, extensionKI = 0.0, extensionKIZone = 0.0, extensionKff = 0.000156,
        extensionSmMaxRPM = 12000.0, extensionSmMaxRPMs = 20000.0, extensionSmMinRPM = 0.0, extensionSmError = 0.1;
    private final double extensionTicksPerInch = 3.8264;
    private double lastExtensionSetReference = 0.0;

    private final double STOP_DEADBAND = 0.25;

    // Values to use as indexers for the position lists
    private final int START_POSITION = 0, MIN_POSITION = 1, MAX_POSITION = 2;

    /**
     * Stores notable arm positions along with methods to update and go to them
     */
    public enum ArmPositions {
        RETRACTED(0.0, 0.0),
        CONE_MEDIUM(15.893, 79.336),
        CUBE_HIGH(18.440, 89.909),
        CUBE_MEDIUM(19.524,14.643),
        CUBE_HYBRID(33.92, 0.0),
        CONE_HYBRID(20.333, 17.405),
        SUBSTATION_CUBE(18.750, 87.313),
        SUBSTATION_CONE_START(10.31, 89.718),
        SUBSTATION_CONE_END(14.619, 69.430),
        GROUND(36.75, 0.5);

        //50.333

        private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();

        public static ArmPositions currentPosition = RETRACTED;
        public static double bump;

        private double pivot;
        private double extension;
        private final double DEADBAND = 0.5;

        ArmPositions(double pivot, double extension) {
            this.pivot = pivot;
            this.extension = extension;
        }

        public double getPivot() {
            return pivot;
        }

        public double getExtension() {
            return extension;
        }

        public static void bumpPivotUp() {
            currentPosition.pivot += bump;
            currentPosition.goTo();
        }

        public static void bumpPivotDown() {
            currentPosition.pivot -= bump;
            currentPosition.goTo();
        }

        public static void bumpExtensionUp() {
            currentPosition.extension += bump;
            currentPosition.goTo();
        }

        public static void bumpExtensionDown() {
            currentPosition.extension -= bump;
            currentPosition.goTo();
        }

        public void goTo() {
            armSubsystem.setPivotPosition(pivot);
            armSubsystem.setExtensionPosition(extension);
            currentPosition = this;
        }

        public boolean isInPosition() {
            return Math.abs(armSubsystem.getPivotPosition() - pivot) < DEADBAND &&
                    Math.abs(armSubsystem.getExtensionPosition() - extension) < DEADBAND;
        }
    }


    /**
     * The Singleton instance of this ArmSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static ArmSubsystem INSTANCE = new ArmSubsystem();

    /**
     * Returns the Singleton instance of this ArmSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code ArmSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static ArmSubsystem getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this ArmSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private ArmSubsystem() {
        // create and initialize the motor that provides forward arm support
        forwardPivot = SparkNeo.factory(Constants.CAN_Devices.PIVOT_FORWARD_SUPPORT_MOTOR);
        forwardPivot.startConfig();
        forwardPivot.setCurrentLimit(SparkNeo.UseType.POSITION, SparkNeo.BreakerAmps.Amps40);
        forwardPivot.setPositionPID(pivotPosKp, pivotPosKi, pivotPosKiZone, pivotPosKff);
        forwardPivot.setSmartMotion(pivotSmKp, pivotSmKi, pivotSmKiZone, pivotSmKff,
                pivotSmMaxRPM, pivotSmMaxRPMs, pivotSmMinRPM, pivotSmError);
        forwardPivot.setIdleMode(CANSparkMax.IdleMode.kBrake);
        forwardPivot.endConfig();

        // create and initialize the motor that provides backward arm support
        backwardPivot = SparkNeo.factory(Constants.CAN_Devices.PIVOT_BACKWARD_SUPPORT_MOTOR);
        backwardPivot.startConfig();
        backwardPivot.setDirection(SparkNeo.Direction.REVERSE);
        backwardPivot.setCurrentLimit(SparkNeo.UseType.POSITION, SparkNeo.BreakerAmps.Amps40);
        backwardPivot.setPositionPID(pivotPosKp, pivotPosKi, pivotPosKiZone, pivotPosKff);
        backwardPivot.setSmartMotion(pivotSmKp, pivotSmKi, pivotSmKiZone, pivotSmKff,
                pivotSmMaxRPM, pivotSmMaxRPMs, pivotSmMinRPM, pivotSmError);
        backwardPivot.setIdleMode(CANSparkMax.IdleMode.kBrake);
        backwardPivot.endConfig();

        // create and initialize the arm extension motor
        extend = SparkNeo.factory(Constants.CAN_Devices.ARM_EXTENSION_MOTOR);
        extend.startConfig();
        extend.setCurrentLimit(SparkNeo.UseType.POSITION, SparkNeo.BreakerAmps.Amps40);
        extend.setSmartMotion(extensionKP, extensionKI, extensionKIZone, extensionKff,
                extensionSmMaxRPM, extensionSmMaxRPMs, extensionSmMinRPM, extensionSmError);
        extend.setIdleMode(CANSparkMax.IdleMode.kBrake);
        extend.endConfig();

        // initialize the encoders - assuming the arm is at the correct starting position.
        forwardPivot.setEncoderPosition(pivotPositions[START_POSITION]);
        backwardPivot.setEncoderPosition(pivotPositions[START_POSITION]);
        extend.setEncoderPosition(extensionPositions[START_POSITION]);
    }

    public void enableInit() {
        if (enableInit) {
            return;
        }

        double startTime = Timer.getFPGATimestamp();
        System.out.println("****************************************************************");
        System.out.println("ENABLE INIT STARTED: " + startTime);
        System.out.println("****************************************************************");

        forwardPivot.setEncoderPosition(pivotPositions[START_POSITION]);
        backwardPivot.setEncoderPosition(pivotPositions[START_POSITION]);

        // Lock the forward (supporting) motor to start pos.
        forwardPivot.setTargetPosition(pivotPositions[START_POSITION]);
        System.out.println("****************************************************************");
        System.out.println(String.format("TIME: %f; support = %f; tension = %f", Timer.getFPGATimestamp()-startTime,
                forwardPivot.getEncoderPosition(), backwardPivot.getEncoderPosition()));
        // run the backwards motor at 1.2 volts, just enough to tension
        backwardPivot.sparkMaxPID.setReference(1.2, CANSparkMax.ControlType.kVoltage,
                SparkNeo.PIDtype.SMART_MOTION.slotId);

        while(true) {
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                continue;
            }
            double currentPos = forwardPivot.getEncoderPosition();

            System.out.println(String.format("TIME: %f; support = %f; tension = %f;",
                    Timer.getFPGATimestamp()-startTime, currentPos, backwardPivot.getEncoderPosition()));

            if (currentPos > 0.0) {
                break;
            }
        }

        backwardPivot.setEncoderPosition(pivotPositions[START_POSITION]);

        System.out.println(String.format("END TIME: %f; support = %f; tension = %f", Timer.getFPGATimestamp()-startTime,
                forwardPivot.getEncoderPosition(), backwardPivot.getEncoderPosition()));
        System.out.println("****************************************************************");
        System.out.println("****************************************************************");
        System.out.println("****************************************************************");

        // Removed play, encoders are reset, tensioning may have displaced the armn, so reset the arm to the start position
        forwardPivot.setSmartMotionTarget(pivotPositions[START_POSITION]);
        backwardPivot.setSmartMotionTarget(pivotPositions[START_POSITION]);

        lastPivotSetReference = pivotPositions[START_POSITION];
        enableInit = true;
    }

    /**
     * Method to get the position of the pivot motors. Gets the average position of the two motors controlling pivot.
     * @return The current pivot position
     */
    public double getPivotPosition() {
        return (forwardPivot.getEncoderPosition() + backwardPivot.getEncoderPosition())/2;
    }

    /**
     * Method to send the position of the arm extension motor
     * @return Double of the current location of the extension motor
     */
    public double getExtensionPosition() {
        return extend.getEncoderPosition();
    }

    /**
     * Sets the pivot motor to position, but will clip the value to stay within min and max positions
     * @param position encoder tick to go to
     */
    public void setPivotPosition(double position) {
        double clippedPosition = Utl.clip(position, pivotPositions[MIN_POSITION], pivotPositions[MAX_POSITION]);
        if(A05Constants.getPrintDebug() && clippedPosition != position) {
            System.out.println("Pivot motor was requested to go to position: " + position + " but was outside limits");
        }
        forwardPivot.setSmartMotionTarget(clippedPosition);
        backwardPivot.setSmartMotionTarget(clippedPosition);
        lastPivotSetReference =  clippedPosition;
    }

    public void setPivotPositionDelta(double delta) {
        setPivotPosition(getPivotPosition() + delta);
    }

    /**
     * Sets the speed of the extension motor but stops the motor if the position exceeds the minimum or maximum position
     * @param power speed between -1 and 1 for the motor.
     */
    public void setExtensionPower(double power) {
        if(getExtensionPosition() > extensionPositions[MAX_POSITION] - STOP_DEADBAND && power > 0) {
            setExtensionPosition(extensionPositions[MAX_POSITION]);
        }
        else if(getExtensionPosition() < extensionPositions[MIN_POSITION] + STOP_DEADBAND && power < 0) {
            setExtensionPosition(extensionPositions[MIN_POSITION]);
        } else {
            extend.sparkMax.set(power);
        }
    }

    /**
     * Sets the extension motor to position, but will clip the value to stay within min and max positions
     * @param position encoder tick to go to
     */
    public void setExtensionPosition(double position) {
        double clippedPosition = Utl.clip(position, extensionPositions[MIN_POSITION], extensionPositions[MAX_POSITION]);
        if(A05Constants.getPrintDebug() && clippedPosition != position) {
            System.out.println("Extension motor was requested to go to position: " + position + " but was outside limits");
        }
        extend.setSmartMotionTarget(clippedPosition);
    }

    public void setExtensionPositionDelta(double delta) {
        setExtensionPosition(extend.getEncoderPosition() + delta);
    }

    /**
     * Stops all motors. Does not persist so if something is periodically powering motors, this won't work.
     */
    public void stopAllMotors() {
        extend.sparkMax.stopMotor();
        forwardPivot.sparkMax.stopMotor();
        backwardPivot.sparkMax.stopMotor();
    }

    public boolean isManualControl() {
        return manualControl;
    }

    public void toggleManualControl() {
        manualControl = !manualControl;
    }

    public void periodic() {
        ArmGeometry.ArmPosition position = new ArmGeometry.ArmPosition(
                getPivotPosition(),getExtensionPosition());
        SmartDashboard.putNumber("pivot", position.getPivotPosition());
        SmartDashboard.putNumber("ext.", position.getExtensionPosition());
//        Point2D.Double pt = ArmGeometry.getArmLocationFromPositions(
//                position.getPivotPosition(),position.getExtensionPosition());
//        SmartDashboard.putNumber("arm X", pt.x);
//        SmartDashboard.putNumber("arm Y", pt.y);
//        boolean valid = position.clipToValidInPlay();
//        SmartDashboard.putBoolean("arm valid", valid);

    }
}

