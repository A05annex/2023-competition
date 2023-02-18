package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawSubsystem extends SubsystemBase {


    private final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
            Constants.PNEUMATICS_OUT, Constants.PNEUMATICS_IN);

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
    private ClawSubsystem() {}

    public void open() {
        solenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void close() {
        solenoid.set(DoubleSolenoid.Value.kOff);
    }

    public void bleed() {
        solenoid.set(DoubleSolenoid.Value.kForward);
    }
}

