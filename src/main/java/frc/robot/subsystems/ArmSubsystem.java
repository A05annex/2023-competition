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
    private final double[] pivotPositions = {0.0, -10, 45};
    private final double pivotKP = 0.1, pivotKI = 0.0, pivotKIZone = 0.0;
    private final double pivotTicksPerRotation = 120.475;


    // Declaring everything for the extension motor
    private final CANSparkMax m_extension = new CANSparkMax(Constants.CAN_Devices.ARM_EXTENSION_MOTOR,
            CANSparkMaxLowLevel.MotorType.kBrushless);
    private final RelativeEncoder m_extensionEncoder = m_extension.getEncoder();
    private final SparkMaxPIDController m_extensionPID = m_extension.getPIDController();
    // Array of positions. [starting position, min position, max position]
    private final double[] extensionPositions = {0.0, 0.0, 205.0};
    private final double extensionKP = 0.1, extensionKI = 0.0, extensionKIZone = 0.0;
    private final double extensionTicksPerInch = 3.939;

    private final double STOP_DEADBAND = 0.25;

    // Values to use as indexers for the position lists
    private final int START_POSITION = 0, MIN_POSITION = 1, MAX_POSITION = 2;


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
     * @param power speed between 0 and 1 of the motor.
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

    public void setPivotPosition(double position) {
        double clippedPosition = Utl.clip(position, pivotPositions[MIN_POSITION], pivotPositions[MAX_POSITION]);
        if(A05Constants.getPrintDebug() && clippedPosition != position) {
            System.out.println("Pivot motor was requested to go to position: " + position + " but was outside limits");
        }
        m_pivotPID.setReference(clippedPosition, CANSparkMax.ControlType.kPosition);
    }



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

    public void setExtensionPosition(double position) {
        double clippedPosition = Utl.clip(position, extensionPositions[MIN_POSITION], extensionPositions[MAX_POSITION]);
        if(A05Constants.getPrintDebug() && clippedPosition != position) {
            System.out.println("Extension motor was requested to go to position: " + position + " but was outside limits");
        }
        m_extensionPID.setReference(clippedPosition, CANSparkMax.ControlType.kPosition);
    }

    /**
     * Reads the encoder position of the pivot motor and does trig to find the max extension of the arm that stays in
     * the 48-inch extension limit
     * @return position in encoder ticks that the extension motor should limit itself to
     */
    public double pivotToExtension() {
        AngleD angle = new AngleD().setDegrees((getPivotPosition() / pivotTicksPerRotation) * 360);
        double distInches = 44/angle.sin();
        return 200 - (distInches * extensionTicksPerInch * 0.9);
    }

    /**
     * Reads the encoder position of the extension motor and does trig to find the max angle of the arm that stays in
     * the 48-inch extension limit
     * @return position in encoder ticks that the pivot motor should limit itself to
     */
    public double extensionToPivot() {
        double distMeters = getExtensionPosition() / extensionTicksPerInch;
        AngleD angle = new AngleD().asin(44/distMeters);
        return 200.0 - (angle.getRadians() / AngleConstantD.TWO_PI.getRadians()) * pivotTicksPerRotation;
    }


    @Override
    public void periodic() {}
}

