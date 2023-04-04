package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.a05annex.util.Utl;

public class CollectorSubsystem extends SubsystemBase {

    private final CANSparkMax cone = new CANSparkMax(Constants.CAN_Devices.CONE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final double coneKp = 0.0001, coneKi = 0.0, coneKiZone = 0.0;

    private final CANSparkMax cube = new CANSparkMax(Constants.CAN_Devices.CUBE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final double cubeKp = 0.0001, cubeKi = 0.0, cubeKiZone = 0.0, cubeKff = 0.0;

    private final SparkMaxPIDController conePID = cone.getPIDController();
    private final SparkMaxPIDController cubePID = cube.getPIDController();

    private final double maxSpeed = 5000.0;

    private final static CollectorSubsystem INSTANCE = new CollectorSubsystem();

    public static CollectorSubsystem getInstance() {
        return INSTANCE;
    }

    private CollectorSubsystem() {
        if (Constants.getSparkConfigFromFactoryDefaults()) {
            cone.restoreFactoryDefaults();
            cone.setSmartCurrentLimit(20, 15, 9000);
            cone.setInverted(true);
            cube.restoreFactoryDefaults();
            cube.setSmartCurrentLimit(20, 15, 9000);
            //cube.setIdleMode(CANSparkMax.IdleMode.kBrake);
            setPID(conePID, coneKp, coneKi, coneKiZone, cubeKff, 0);
            setPID(conePID, 0.1, 0.0, 0.0, 0.0, 1);
            setPID(cubePID, cubeKp, cubeKi, cubeKiZone, cubeKff, 0);
            setPID(cubePID, 0.1, 0.0, 0.0, 0.0, 1);

            if (Constants.getSparkBurnConfig()) {
                cone.burnFlash();
                cube.burnFlash();
            }
        }
        disableUnusedFrames(cone);
        disableUnusedFrames(cube);
    }

    private void disableUnusedFrames(CANSparkMax motor) {
        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3,500);
        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus4,500);
        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus5,500);
        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus6,500);
    }

    private void setPID(SparkMaxPIDController pid, double kp, double ki, double kiZone, double kff, int slot) {
        pid.setP(kp, slot);
        pid.setI(ki, slot);
        pid.setIZone(kiZone, slot);
        pid.setFF(kff, slot);
        pid.setOutputRange(-1.0, 1.0, slot);
    }


    public void spinConeMotor() {
        conePID.setReference(maxSpeed, CANSparkMax.ControlType.kVelocity, 0);
    }

    public void stopConeMotor() {
        conePID.setReference(cone.getEncoder().getPosition(), CANSparkMax.ControlType.kPosition, 1);
    }


    public void spinCubeMotor() {
        cubePID.setReference(maxSpeed, CANSparkMax.ControlType.kVelocity, 0);
    }

    public void stopCubeMotor() {
        cubePID.setReference(cube.getEncoder().getPosition(), CANSparkMax.ControlType.kPosition, 1);
    }

    public void spinAtSpeed(double speed) {
        double clippedSpeed = Utl.clip(speed, -maxSpeed, maxSpeed);

        conePID.setReference(clippedSpeed, CANSparkMax.ControlType.kVelocity, 0);
        cubePID.setReference(clippedSpeed, CANSparkMax.ControlType.kVelocity, 0);
    }

    public void spin() {
        conePID.setReference(maxSpeed, CANSparkMax.ControlType.kVelocity, 0);
        cubePID.setReference(maxSpeed, CANSparkMax.ControlType.kVelocity, 0);
    }

    public void stop() {
        cubePID.setReference(cube.getEncoder().getPosition(), CANSparkMax.ControlType.kPosition, 1);
        conePID.setReference(cone.getEncoder().getPosition(), CANSparkMax.ControlType.kPosition, 1);
    }
}

