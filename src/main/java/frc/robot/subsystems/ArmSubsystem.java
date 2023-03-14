package frc.robot.subsystems;


import com.revrobotics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.a05annex.frc.A05Constants;
import org.a05annex.util.AngleConstantD;
import org.a05annex.util.AngleD;
import org.a05annex.util.Utl;

public class ArmSubsystem extends SubsystemBase {
    private boolean enableInit = false;

    // Declaring everything for the forward support pivot motor (the motor that is supporting the arm when
    // it is forward, or with the claw towards the front of the robot)
    private final CANSparkMax forwardSupportPivot = new CANSparkMax(Constants.CAN_Devices.PIVOT_FORWARD_SUPPORT_MOTOR,
            CANSparkMaxLowLevel.MotorType.kBrushless);
    private final RelativeEncoder forwardEncoder = forwardSupportPivot.getEncoder();
    private final SparkMaxPIDController forwardPID = forwardSupportPivot.getPIDController();

    // Declaring everything for the backward support pivot motor (the motor that is supporting the arm when
    // it is backwards, or with the claw towards the back of the robot)
    private final CANSparkMax backwardSupportPivot = new CANSparkMax(Constants.CAN_Devices.PIVOT_BACKWARD_SUPPORT_MOTOR,
            CANSparkMaxLowLevel.MotorType.kBrushless);
    private final RelativeEncoder backwardEncoder = backwardSupportPivot.getEncoder();
    private final SparkMaxPIDController backwardPID = backwardSupportPivot.getPIDController();

    //TODO: make this more readable
    //Array of positions. [starting position, min position, max position]
    private final double[] pivotPositions = {0.0, -45, 45};

    // These are the parameters for smart move on the pivot motors. Since we want the motors to move in sync with
    // each other we want exactly the same smart motion parameters for each
    private final double
            pvtSmKp = 0.00005, pvtSmKff = 0.000156, pvtSmMaxRPM = 2000.0, pvtSmMaxRPMs = 2000.0, pvtSmMaxErr = 0.1;
    // These are the parameters for position control of the arm. One of the motors will be the supporting arm
    // (depends on whether the arm is forward or backward with respect to the robot. The supporting motor supports
    // the arm - which is why it has a high Kp and Ki. The tension arm is only resisting bounce, which is why it
    // has a Kp that is a fraction of the support Kp. The tensioning motor does not have a Ki because that would
    // build to inifinite pressure against the supporting motor.
    private final double supportPosKp = 0.1, supportPosKi = 0.0001, supportPosKiZone = pvtSmMaxErr * 4.0;
    private final double tensionPosKP = supportPosKp * 0.10;

    private final double pivotTicksPerRotation = 30.309 * 4; //Reading from 0 to 90 degrees * 4 = full rotation

    /**
     * The last pivot position set using {@link SparkMaxPIDController#setReference(double, CANSparkMax.ControlType)}.
     */
    private double lastPivotSetReference = 0.0;
    /**
     * The last {@link CANSparkMax.ControlType} set for the pivot motor supporting the arm.
     */
    private CANSparkMax.ControlType lastPivotDriveMode = CANSparkMax.ControlType.kPosition;
   /**
     * {@code true} if the arm should transition from smart move to position control when the arm reaches
     * (is within a specified error of) the target position. Otherwise, there should be no control transition.
     */
    private boolean lockPivotAtPosition = false;
    private int cyclesSinceLastPivotMove = 0;

    // Declaring everything for the extension motor
    private final CANSparkMax m_extension = new CANSparkMax(Constants.CAN_Devices.ARM_EXTENSION_MOTOR,
            CANSparkMaxLowLevel.MotorType.kBrushless);
    private final RelativeEncoder m_extensionEncoder = m_extension.getEncoder();
    private final SparkMaxPIDController m_extensionPID = m_extension.getPIDController();
    // Array of positions. [starting position, min position, max position]
    private final double[] extensionPositions = {111.51, 0.0, 111.51};
    private final double extensionKP = 0.00005, extensionKI = 0.0, extensionKIZone = 0.0, extensionKff = 0.000156;
    private final double extensionTicksPerInch = 3.7989887133;

    private final double STOP_DEADBAND = 0.25;

    // Values to use as indexers for the position lists
    private final int START_POSITION = 0, MIN_POSITION = 1, MAX_POSITION = 2;

    /**
     * Stores notable arm positions along with methods to update and go to them
     */
    public enum ArmPositions {
        RETRACTED(0.0, 111.51),
        CONE_HIGH(17.5, 0),
        CONE_MEDIUM(17.3, 69.15),
        CUBE_HIGH(20.21, 14.68),
        CUBE_MEDIUM(24.14,85.96),
        HYBRID(33.92, 111.51),
        SUBSTATION_CUBE(20.9, 43.01),
        SUBSTATION_CONE(20.7142, 46.13);

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
            return Math.abs(armSubsystem.getPivotPosition() - currentPosition.pivot) < DEADBAND &&
                    Math.abs(armSubsystem.getExtensionPosition() - currentPosition.extension) < DEADBAND;
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
        // Initialize the forward support pivot motor
        forwardSupportPivot.restoreFactoryDefaults();
        forwardSupportPivot.setIdleMode(CANSparkMax.IdleMode.kBrake);
        forwardEncoder.setPosition(pivotPositions[START_POSITION]);
        forwardPID.setOutputRange(-1.0, 1.0);
        setSmartMotion(forwardPID, pvtSmMaxRPM, pvtSmMaxRPMs, pvtSmMaxErr);

        // Initialize the forward support pivot motor
        backwardSupportPivot.restoreFactoryDefaults();
        backwardSupportPivot.setInverted(true);
        backwardSupportPivot.setIdleMode(CANSparkMax.IdleMode.kBrake);
        backwardEncoder.setPosition(pivotPositions[START_POSITION]);
        backwardPID.setOutputRange(-1.0, 1.0);
        setSmartMotion(backwardPID, pvtSmMaxRPM, pvtSmMaxRPMs, pvtSmMaxErr);

        // Initialize the extension motor
        m_extension.restoreFactoryDefaults();
        m_extensionEncoder.setPosition(extensionPositions[START_POSITION]);
        m_extensionPID.setOutputRange(-1.0, 1.0);
        setPID(m_extensionPID, extensionKP, extensionKI, extensionKIZone, extensionKff);
        m_extension.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void enableInit() {
        if (enableInit) {
            return;
        }
        // Lock the forward motor (supporting motor when the arm is forward) to start pos.
        setPivotPositionPIDs(forwardPID, backwardPID);
        forwardPID.setReference(pivotPositions[START_POSITION], lastPivotDriveMode);

//        // 0.5 volts, just enough to tension against the forward position.
//        backwardPID.setReference(0.5, CANSparkMax.ControlType.kVoltage);
//        // apply power until the motor stops moving.
//        double lastPos = backwardEncoder.getPosition();
//        while(true) {
//            try {
//                Thread.sleep(50);
//            } catch (InterruptedException e) {
//                continue;
//            }
//            double currentPos = backwardEncoder.getPosition();
//            if(currentPos == lastPos) {
//                break;
//            }
//            lastPos = currentPos;
//        }
//        // Removed play and resetting the backward motor encoder so that it can be held in place with
//        // a constant tension against the forward motor.
//        backwardEncoder.setPosition(pivotPositions[START_POSITION]);


//        // now lock the backward motor to the start position
//        backwardPID.setReference(pivotPositions[START_POSITION], lastPivotDriveMode);
//        lastPivotSetReference = pivotPositions[START_POSITION];
//        lockAtPosition = true;
//        enableInit = true;
    }

    private void setPivotSmartMotionPIDs(SparkMaxPIDController support, SparkMaxPIDController tension) {
        if (lastPivotDriveMode != CANSparkMax.ControlType.kSmartMotion) {
            setPID(support, pvtSmKp, 0.0, 0.0, pvtSmKff);
            setPID(tension, pvtSmKp, 0.0, 0.0, pvtSmKff);
            lastPivotDriveMode = CANSparkMax.ControlType.kSmartMotion;
        }
    }

    private void setPivotPositionPIDs(SparkMaxPIDController support, SparkMaxPIDController tension) {
        // this is called when the motors have been in smart motion control and the position is now being locked
        // by going into position control. Position control K's are way bigger than the velocity control K's used
        // in smart motion. So, I wonder about ordering here - like, will set the K's before setting the desired
        // outcome, the setReference(), result in an unstable condition. I think that could be a problem, so set
        // the ref speed to 0.0
        if (lastPivotDriveMode != CANSparkMax.ControlType.kPosition) {
            support.setReference(0.0, CANSparkMax.ControlType.kVelocity);
            setPID(support, supportPosKp, supportPosKi, supportPosKiZone, 0.0);
            tension.setReference(0.0, CANSparkMax.ControlType.kVelocity);
            setPID(tension, tensionPosKP, 0.0, 0.0, 0.0);
            lastPivotDriveMode = CANSparkMax.ControlType.kPosition;
        }
    }

    /**
     * Sets the PID values of a motor.
     * @param pid Motor that you want to set values for
     * @param kP kP value you wish to apply
     * @param kI kI value you wish to apply
     * @param kIZone Distance away from target to start applying kI
     */
    private void setPID(SparkMaxPIDController pid, double kP, double kI, double kIZone, double kFF) {
        pid.setP(kP);
        pid.setI(kI);
        pid.setIZone(kIZone);
        pid.setFF(kFF);
        pid.setD(0.0);
    }

    private void setSmartMotion(SparkMaxPIDController pid, double maxRPM, double maxRPMs, double maxErr) {
        pid.setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kTrapezoidal, 0);
        pid.setSmartMotionMaxVelocity(maxRPM, 0);
        pid.setSmartMotionMaxAccel(maxRPMs, 0);
        pid.setSmartMotionMinOutputVelocity(0.0, 0);
        pid.setSmartMotionAllowedClosedLoopError(maxErr, 0);
    }

    /**
     * Method to get the current average position of the pivot motors.
     * @return (double) the current average position of the pivot motors.
     */
    public double getPivotPosition() {
        return (forwardEncoder.getPosition() + backwardEncoder.getPosition()) / 2.0;
    }
    /**
     * Method to get the reference set position for the pivot motors
     * @return (double) the current average position of the pivot motors.
     */
    public double getPivotReferencePosition() {
        return lastPivotSetReference;
    }

    /**
     * Method to send the position of the arm extension motor
     * @return Double of the current location of the extension motor
     */
    public double getExtensionPosition() {
        return m_extensionEncoder.getPosition();
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
        setPivotSmartMotionPIDs(forwardPID, backwardPID);
        forwardPID.setReference(clippedPosition, lastPivotDriveMode);
        backwardPID.setReference(clippedPosition, lastPivotDriveMode);
        lastPivotSetReference =  clippedPosition;
        lockPivotAtPosition = true;
    }

    public void setPivotPositionDelta(double delta) {
        double currentPos = (forwardEncoder.getPosition() + backwardEncoder.getPosition()) / 2.0;
        double clippedPosition = Utl.clip(currentPos + delta,
                pivotPositions[MIN_POSITION], pivotPositions[MAX_POSITION]);
        if(A05Constants.getPrintDebug() && clippedPosition != (currentPos + delta)) {
            System.out.println("Pivot motor was requested to go to position: " + (currentPos + delta) +
                    " but was outside limits");
        }
        setPivotSmartMotionPIDs(forwardPID, backwardPID);
        forwardPID.setReference(clippedPosition, lastPivotDriveMode);
        backwardPID.setReference(clippedPosition, lastPivotDriveMode);
        lastPivotSetReference =  clippedPosition;
        lockPivotAtPosition = false;
        cyclesSinceLastPivotMove = 0;
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
            m_extension.set(power);
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
        m_extensionPID.setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kTrapezoidal, 0);
        m_extensionPID.setSmartMotionMaxVelocity(6000.0, 0);
        m_extensionPID.setSmartMotionMaxAccel(10000.0, 0);
        m_extensionPID.setSmartMotionMinOutputVelocity(0.0, 0);
        m_extensionPID.setSmartMotionAllowedClosedLoopError(0.1, 0);
        m_extensionPID.setReference(clippedPosition, CANSparkMax.ControlType.kSmartMotion);
    }

    public void goToCalcPos() {
        setExtensionPosition(pivotToExtension());
    }

    /**
     * Reads the encoder position of the pivot motor and does trig to find the max extension point of the arm that stays
     * in the 48-inch extension limit
     * @return position in encoder ticks that the extension motor should limit itself to
     */
    public double pivotToExtension() {
        AngleD angle = new AngleD().setDegrees((getPivotPosition() / pivotTicksPerRotation) * 360);
        double distInches = 39.5/angle.sin();
        return 200 - (distInches * extensionTicksPerInch * 0.9);
    }

    /**
     * Reads the encoder position of the extension motor and does trig to find the max angle of the arm that stays in
     * the 48-inch extension limit
     * @return position in encoder ticks that the pivot motor should limit itself to
     */
    public double extensionToPivot() {
        double distMeters = getExtensionPosition() / extensionTicksPerInch;
        AngleD angle = new AngleD().asin(39.5/distMeters);
        return 200.0 - (angle.getRadians() / AngleConstantD.TWO_PI.getRadians()) * pivotTicksPerRotation;
    }

    /**
     * Stops all motors. Does not persist so if something is periodically powering motors, this won't work.
     */
    public void stopAllMotors() {
        m_extension.stopMotor();
        backwardSupportPivot.stopMotor();
    }

    public void periodic() {
        ArmPositions.bump = Constants.updateConstant("bump", ArmPositions.bump);
        SmartDashboard.putNumber("pivot", getPivotPosition());
        SmartDashboard.putNumber("ext.", getExtensionPosition());
        if ((lastPivotDriveMode == CANSparkMax.ControlType.kSmartMotion) &&
                (lockPivotAtPosition || (cyclesSinceLastPivotMove > 10))) {
            // if the arm is in smart-motion it is being controlled with a velocity profile. Once
            // we reach the requested position and we have requested a lock or there has been enough
            // time since the last pivot move, we want to lock it with position mode
            double currentPos = (forwardEncoder.getPosition() + backwardEncoder.getPosition()) / 2.0;
            if ((currentPos < (lastPivotSetReference + (2.0 * pvtSmMaxErr))) &&
                    (currentPos > (lastPivotSetReference - (2.0 * pvtSmMaxErr)))) {
                // OK, we are at the target so we want to lock with the correct
                // supporting motor locking the position.
                System.out.println("***********************************************************************");
                System.out.println("***********************************************************************");
                if (currentPos < (-3.0 * pvtSmMaxErr)) {
                    System.out.println("**** ARM AT POSITION - BACKWARD motor is supporting the arm        ****");
                    // the arm is leaning backwards - make the backwards motor the support motor that is running the
//                    setPivotPositionPIDs(backwardPID, forwardPID);
//                    backwardPID.setReference(lastPivotSetReference, lastPivotDriveMode);
//                    forwardPID.setReference(pivotPositions[START_POSITION], lastPivotDriveMode);
//                    lastPivotDriveMode = CANSparkMax.ControlType.kPosition;
                } else {
                    System.out.println("**** ARM AT POSITION - FORWARD motor is supporting the arm         ****");
                    // the arm is leaning forward
//                    setPivotPositionPIDs(forwardPID, backwardPID);
//                    forwardPID.setReference(lastPivotSetReference, lastPivotDriveMode);
//                    backwardPID.setReference(pivotPositions[START_POSITION], lastPivotDriveMode);
//                    lastPivotDriveMode = CANSparkMax.ControlType.kPosition;
                }
                System.out.println("***********************************************************************");
                System.out.println("***********************************************************************");
            }
        }
        cyclesSinceLastPivotMove++;
    }
}

