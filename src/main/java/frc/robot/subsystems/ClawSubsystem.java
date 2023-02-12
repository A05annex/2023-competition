package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawSubsystem extends SubsystemBase {

    private final CANSparkMax m_motor = new CANSparkMax(Constants.CAN_Devices.CLAW_MOTOR,
            CANSparkMaxLowLevel.MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_motor.getEncoder();
    private final SparkMaxPIDController m_motorPID = m_motor.getPIDController();
    // Array of positions. [starting position, open, fully closed, cube, cone]
    private final double[] positions = {0.0, 0.190476, 0.0, 0.095238, 0.0};
    private final double kP = 0.8, kI = 0.01, kIZone = 1.0;

    // Values to use as indexers for the position list
    private final int START_POSITION = 0, OPEN = 1, CLOSED = 2, CUBE = 3, CONE = 4;
    private int currentIndex;

    // With eager singleton initialization, any static variables/fields used in the 
    // constructor must appear before the "INSTANCE" variable so that they are initialized 
    // before the constructor is called when the "INSTANCE" variable initializes.

    /**
     * The Singleton instance of this ClawSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static ClawSubsystem INSTANCE = new ClawSubsystem();

    /**
     * Returns the Singleton instance of this ClawSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code ClawSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static ClawSubsystem getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this ClawSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private ClawSubsystem() {
        setPID(m_motorPID, kP, kI, kIZone);
        initializeEncoders();
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

    public void initializeEncoders() {
        m_encoder.setPosition(positions[START_POSITION]);
        currentIndex = START_POSITION;
    }

    public double getPosition() {
        return m_encoder.getPosition();
    }

    public void goToOpen() {
        currentIndex = OPEN;
        moveToCurrentIndex();
    }

    public void goToClosed() {
        currentIndex = CLOSED;
        moveToCurrentIndex();
    }

    public void goToCube() {
        currentIndex = CUBE;
        moveToCurrentIndex();
    }

    public void goToCone() {
        currentIndex = CONE;
        moveToCurrentIndex();
    }

    private void moveToCurrentIndex() {
        m_motorPID.setReference(positions[currentIndex], CANSparkMax.ControlType.kPosition);
    }

    public void stop() {
        m_motor.stopMotor();
    }
}

