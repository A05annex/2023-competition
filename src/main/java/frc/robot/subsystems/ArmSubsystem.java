package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.a05annex.frc.A05Constants;
import org.a05annex.util.AngleConstantD;
import org.a05annex.util.AngleD;
import org.a05annex.util.Utl;

public class ArmSubsystem extends SubsystemBase {
    // Declaring everything for the pivot motor
    private final CANSparkMax m_pivot = new CANSparkMax(Constants.CAN_Devices.ARM_PIVOT_MOTOR,
            CANSparkMaxLowLevel.MotorType.kBrushless);
    private final RelativeEncoder m_pivotEncoder = m_pivot.getEncoder();
    private final SparkMaxPIDController m_pivotPID = m_pivot.getPIDController();
    // Array of positions. [starting position, min position, max position]
    private final double[] pivotPositions = {0.0, -45, 45};
    private final double pivotKP = 0.1, pivotKI = 0.0, pivotKIZone = 0.0;
    private final double pivotTicksPerRotation = 30.309 * 4; //Reading from 0 to 90 degrees * 4 = full rotation


    // Declaring everything for the extension motor
    private final CANSparkMax m_extension = new CANSparkMax(Constants.CAN_Devices.ARM_EXTENSION_MOTOR,
            CANSparkMaxLowLevel.MotorType.kBrushless);
    private final RelativeEncoder m_extensionEncoder = m_extension.getEncoder();
    private final SparkMaxPIDController m_extensionPID = m_extension.getPIDController();
    // Array of positions. [starting position, min position, max position]
    private final double[] extensionPositions = {247.8, 0.0, 247.8};
    private final double extensionKP = 0.3, extensionKI = 0.0, extensionKIZone = 0.0;
    private final double extensionTicksPerInch = 3.7989887133;

    private final double STOP_DEADBAND = 0.25;

    // Values to use as indexers for the position lists
    private final int START_POSITION = 0, MIN_POSITION = 1, MAX_POSITION = 2;

    /**
     * Stores notable arm positions along with methods to update and go to them
     */
    public enum ArmPositions {
        RETRACTED(0.0, 247.8),
        CONE_HIGH(15.5, 27.0),
        CONE_MEDIUM(16.0, 157.0),
        CUBE_HIGH(17.0, 52.0),
        CUBE_MEDIUM(20.5,179.49),
        HYBRID(33.92, 247.8);

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
            currentPosition.extension += bump;
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
        initializeEncoders();

        setPID(m_pivotPID, pivotKP, pivotKI, pivotKIZone);
        setPID(m_extensionPID, extensionKP, extensionKI, extensionKIZone);

        m_pivotPID.setOutputRange(-0.2, 0.2);
    }


    /**
     * Sets the PID values of a motor.
     * @param motor Motor that you want to set values for
     * @param kP kP value you wish to apply
     * @param kI kI value you wish to apply
     * @param kIZone Distance away from target to start applying kI
     */
    private void setPID(SparkMaxPIDController motor, double kP, double kI, double kIZone) {
        motor.setP(kP);
        motor.setI(kI);
        motor.setIZone(kIZone);
        motor.setFF(0.0);
        motor.setD(0.0);
    }

    /**
     * Resets all encoders to their start positions
     */
    public void initializeEncoders() {
        m_pivotEncoder.setPosition(pivotPositions[START_POSITION]);
        m_extensionEncoder.setPosition(extensionPositions[START_POSITION]);
    }

    /**
     * Method to send the position of the pivot motor
     * @return Double of the current location of the pivot motor
     */
    public double getPivotPosition() {
        return m_pivotEncoder.getPosition();
    }

    /**
     * Method to send the position of the arm extension motor
     * @return Double of the current location of the extension motor
     */
    public double getExtensionPosition() {
        return m_extensionEncoder.getPosition();
    }

    /**
     * Sets the speed of the pivot motor but stops the motor if the position exceeds the minimum or maximum position
     * @param power speed between -1 and 1 for the motor.
     */
    public void setPivotPower(double power) {
        if(getPivotPosition() > pivotPositions[MAX_POSITION] - STOP_DEADBAND && power > 0) {
            setPivotPosition(pivotPositions[MAX_POSITION]);
        }
        else if(getPivotPosition() < pivotPositions[MIN_POSITION] + STOP_DEADBAND && power < 0) {
            setPivotPosition(pivotPositions[MIN_POSITION]);
        } else {
            m_pivot.set(power * 0.3);
        }
        //setExtensionPosition(pivotToExtension());
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
        m_pivotPID.setReference(clippedPosition, CANSparkMax.ControlType.kPosition);
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
            m_extension.set(power * 0.5);
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
        m_extensionPID.setReference(clippedPosition, CANSparkMax.ControlType.kPosition);
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
        m_pivot.stopMotor();
    }

    public void periodic() {
        ArmPositions.bump = Constants.updateConstant("bump", ArmPositions.bump);
    }
}

