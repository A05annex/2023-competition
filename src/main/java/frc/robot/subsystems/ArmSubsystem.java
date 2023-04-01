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

    private boolean manualControl = false;

    // Declaring everything for the forward support pivot motor
    private final CANSparkMax forwardSupportPivot = new CANSparkMax(Constants.CAN_Devices.PIVOT_FORWARD_SUPPORT_MOTOR,
            CANSparkMaxLowLevel.MotorType.kBrushless);
    private final RelativeEncoder forwardEncoder = forwardSupportPivot.getEncoder();
    private final SparkMaxPIDController forwardPID = forwardSupportPivot.getPIDController();

    // Declaring everything for the backward support pivot motor
    private final CANSparkMax backwardSupportPivot = new CANSparkMax(Constants.CAN_Devices.PIVOT_BACKWARD_SUPPORT_MOTOR,
            CANSparkMaxLowLevel.MotorType.kBrushless);
    private final RelativeEncoder backwardEncoder = backwardSupportPivot.getEncoder();
    private final SparkMaxPIDController backwardPID = backwardSupportPivot.getPIDController();

    //TODO: make this more readable
    //Array of positions. [starting position, min position, max position]
    private final double[] pivotPositions = {0.0, -40, 45};

    private final double supportSmKp = 0.00005, supportSmKi = 0.000, supportSmKiZone = 0.0, supportSmKff = 0.000156;
    private final double supportPosKp = 0.00005, supportPosKi = 0.000, supportPosKiZone = 0.0, supportPosKff = 0.000156;
    private final double tensionKP = 0.00005, tensionKff = 0.000156;

    private final double pivotTicksPerRotation = 30.309 * 4; //Reading from 0 to 90 degrees * 4 = full rotation

    private double lastPivotSetReference = 0.0;
    private CANSparkMax.ControlType lastPivotDriveMode = CANSparkMax.ControlType.kPosition;

    // Declaring everything for the extension motor
    private final CANSparkMax extension = new CANSparkMax(Constants.CAN_Devices.ARM_EXTENSION_MOTOR,
            CANSparkMaxLowLevel.MotorType.kBrushless);
    private final RelativeEncoder extensionEncoder = extension.getEncoder();
    private final SparkMaxPIDController extensionPID = extension.getPIDController();
    // Array of positions. [starting position, min position, max position]
    private final double[] extensionPositions = {0.0, 0.125, 170.375};
    private final double extensionKP = 0.00005, extensionKI = 0.0, extensionKIZone = 0.0, extensionKff = 0.000156;
    private final double extensionTicksPerInch = 3.7989887133;

    private final double STOP_DEADBAND = 0.25;

    // Values to use as indexers for the position lists
    private final int START_POSITION = 0, MIN_POSITION = 1, MAX_POSITION = 2;

    /**
     * Stores notable arm positions along with methods to update and go to them
     */
    public enum ArmPositions {
        RETRACTED(0.0, 0.0),
        CONE_HIGH(17.5, 170.375),
        CONE_MEDIUM(15.893, 79.336),
        CUBE_HIGH(18.440, 89.909),
        CUBE_MEDIUM(20.488,25.262),
        HYBRID(33.92, 0.0),
        SUBSTATION_CUBE(14.024, 88.766),
        SUBSTATION_CONE(20.7142, 65.38),
        GROUND(36.333, 0.5);

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
        forwardEncoder.setPosition(pivotPositions[START_POSITION]);
        forwardPID.setOutputRange(-1.0, 1.0);
        forwardSupportPivot.setIdleMode(CANSparkMax.IdleMode.kBrake);
        forwardSupportPivot.setSmartCurrentLimit(60,20, 2000);

        // Initialize the forward support pivot motor
        backwardSupportPivot.restoreFactoryDefaults();
        backwardSupportPivot.setInverted(true);
        backwardEncoder.setPosition(pivotPositions[START_POSITION]);
        backwardPID.setOutputRange(-1.0, 1.0);
        backwardSupportPivot.setIdleMode(CANSparkMax.IdleMode.kBrake);
        backwardSupportPivot.setSmartCurrentLimit(60,20, 2000);

        setPivotSmartMotionPIDs(forwardPID, backwardPID);

        // Initialize the extension motor
        extension.restoreFactoryDefaults();
        extension.setInverted(true);
        extensionEncoder.setPosition(extensionPositions[START_POSITION]);
        extensionPID.setOutputRange(-1.0, 1.0);
        setPID(extensionPID, extensionKP, extensionKI, extensionKIZone, extensionKff);
        extension.setIdleMode(CANSparkMax.IdleMode.kBrake);
        backwardSupportPivot.setSmartCurrentLimit(60,20, 2000);
    }

    public void enableInit() {
        if (enableInit) {
            return;
        }
        // Lock the forward (supporting) motor to start pos.
        forwardPID.setReference(pivotPositions[START_POSITION], CANSparkMax.ControlType.kSmartMotion);
        // 0.5 amps, just enough to tension
        backwardPID.setReference(0.5, CANSparkMax.ControlType.kVoltage);

        double lastPos = backwardEncoder.getPosition();
        while(true) {
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                continue;
            }
            double currentPos = backwardEncoder.getPosition();
            if(currentPos == lastPos) {
                break;
            }
            lastPos = currentPos;
        }
        //Removed play and resetting the backward encoder so it can be held in place
        backwardEncoder.setPosition(pivotPositions[START_POSITION]);
        forwardPID.setReference(pivotPositions[START_POSITION], CANSparkMax.ControlType.kSmartMotion);
        backwardPID.setReference(pivotPositions[START_POSITION], CANSparkMax.ControlType.kSmartMotion);

        lastPivotSetReference = pivotPositions[START_POSITION];
        enableInit = true;
    }

    public boolean isInitialized() {
        return enableInit;
    }

    private void setPivotSmartMotionPIDs(SparkMaxPIDController support, SparkMaxPIDController tension) {
        setPID(tension, tensionKP, 0.0, 0.0, tensionKff);
        setPID(support, supportSmKp, supportSmKi, supportSmKiZone, supportSmKff);

        support.setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kTrapezoidal, 0);
        support.setSmartMotionMaxVelocity(2000.0, 0);
        support.setSmartMotionMaxAccel(1000.0, 0);
        support.setSmartMotionMinOutputVelocity(0.0, 0);
        support.setSmartMotionAllowedClosedLoopError(0.1, 0);

        tension.setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kTrapezoidal, 0);
        tension.setSmartMotionMaxVelocity(2000.0, 0);
        tension.setSmartMotionMaxAccel(1000.0, 0);
        tension.setSmartMotionMinOutputVelocity(0.0, 0);
        tension.setSmartMotionAllowedClosedLoopError(0.1, 0);
    }

    private void setPivotPosPIDs(SparkMaxPIDController support, SparkMaxPIDController tension) {
        setPID(tension, tensionKP, 0.0, 0.0, tensionKff);
        setPID(support, supportSmKp, supportSmKi, supportSmKiZone, supportSmKff);

        support.setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kTrapezoidal, 0);
        support.setSmartMotionMaxVelocity(2000.0, 0);
        support.setSmartMotionMaxAccel(1000.0, 0);
        support.setSmartMotionMinOutputVelocity(0.0, 0);
        support.setSmartMotionAllowedClosedLoopError(0.1, 0);

        tension.setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kTrapezoidal, 0);
        tension.setSmartMotionMaxVelocity(2000.0, 0);
        tension.setSmartMotionMaxAccel(1000.0, 0);
        tension.setSmartMotionMinOutputVelocity(0.0, 0);
        tension.setSmartMotionAllowedClosedLoopError(0.1, 0);
    }

    /**
     * Sets the PID values of a motor.
     * @param motor Motor that you want to set values for
     * @param kP kP value you wish to apply
     * @param kI kI value you wish to apply
     * @param kIZone Distance away from target to start applying kI
     */
    private void setPID(SparkMaxPIDController motor, double kP, double kI, double kIZone, double kFF) {
        motor.setP(kP);
        motor.setI(kI);
        motor.setIZone(kIZone);
        motor.setFF(kFF);
        motor.setD(0.0);
    }

    /**
     * Method to send the position of the pivot motor
     * @return Double of the current location of the pivot motor
     */
    public double getPivotPosition() {
        return (forwardEncoder.getPosition() + backwardEncoder.getPosition())/2;
    }

    /**
     * Method to send the position of the arm extension motor
     * @return Double of the current location of the extension motor
     */
    public double getExtensionPosition() {
        return extensionEncoder.getPosition();
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
        forwardPID.setReference(clippedPosition, CANSparkMax.ControlType.kSmartMotion);
        backwardPID.setReference(clippedPosition, CANSparkMax.ControlType.kSmartMotion);
        lastPivotSetReference =  clippedPosition;

    }

    public void setPivotPositionDelta(double delta) {
        double currentPos = (forwardEncoder.getPosition() + backwardEncoder.getPosition())/2;
        double clippedPosition = Utl.clip(currentPos + delta,
                pivotPositions[MIN_POSITION], pivotPositions[MAX_POSITION]);
        if(A05Constants.getPrintDebug() && clippedPosition != (currentPos + delta)) {
            System.out.println("Pivot motor was requested to go to position: " + (currentPos + delta) +
                    " but was outside limits");
        }
        forwardPID.setReference(clippedPosition, CANSparkMax.ControlType.kSmartMotion);
        backwardPID.setReference(clippedPosition, CANSparkMax.ControlType.kSmartMotion);
        lastPivotSetReference =  clippedPosition;
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
            extension.set(power);
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
        extensionPID.setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kTrapezoidal, 0);
        extensionPID.setSmartMotionMaxVelocity(12000, 0);
        extensionPID.setSmartMotionMaxAccel(20000.0, 0);
        extensionPID.setSmartMotionMinOutputVelocity(0.0, 0);
        extensionPID.setSmartMotionAllowedClosedLoopError(0.1, 0);
        extensionPID.setReference(clippedPosition, CANSparkMax.ControlType.kSmartMotion);
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
        extension.stopMotor();
        backwardSupportPivot.stopMotor();
    }

    public boolean isManualControl() {
        return manualControl;
    }

    public void setManualControl(boolean isManual) {
        manualControl = isManual;
    }

    public void periodic() {
        ArmPositions.bump = Constants.updateConstant("bump", ArmPositions.bump);
        SmartDashboard.putNumber("pivot", getPivotPosition());
        SmartDashboard.putNumber("ext.", getExtensionPosition());
    }
}

