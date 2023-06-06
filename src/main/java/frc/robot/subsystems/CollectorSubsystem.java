package frc.robot.subsystems;


//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel;
//import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.a05annex.frc.subsystems.SparkNeo;
import org.a05annex.frc.subsystems.SparkNeo550;
import org.a05annex.util.Utl;

public class CollectorSubsystem extends SubsystemBase {

    private final SparkNeo550 cone = SparkNeo550.factory(Constants.CAN_Devices.CONE_MOTOR);
//    private final CANSparkMax oldCone = new CANSparkMax(Constants.CAN_Devices.CONE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final double coneKp = 0.0001, coneKi = 0.0, coneKiZone = 0.0;

    private final SparkNeo550 cube = SparkNeo550.factory(Constants.CAN_Devices.CUBE_MOTOR);
//    private final CANSparkMax oldCube = new CANSparkMax(Constants.CAN_Devices.CUBE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final double cubeKp = 0.0001, cubeKi = 0.0, cubeKiZone = 0.0, cubeKff = 0.0;

//    private final SparkMaxPIDController conePID = oldCone.getPIDController();
//    private final SparkMaxPIDController cubePID = oldCube.getPIDController();

    private final double maxSpeed = 5000.0;

    private final static CollectorSubsystem INSTANCE = new CollectorSubsystem();

    public static CollectorSubsystem getInstance() {
        return INSTANCE;
    }

    private CollectorSubsystem() {
        cone.startConfig();
        cone.setCurrentLimit(SparkNeo.UseType.POSITION, SparkNeo.BreakerAmps.Amps40);
        cone.setDirection(SparkNeo.Direction.REVERSE);
        cone.setRpmPID(coneKp, coneKi, coneKiZone, cubeKff);
        cone.setPositionPID(0.1, 0.0, 0.0, 0.0);
        cone.endConfig();

        cube.startConfig();
        cube.setCurrentLimit(SparkNeo.UseType.POSITION, SparkNeo.BreakerAmps.Amps40);
        cube.setRpmPID(cubeKp, cubeKi, cubeKiZone, cubeKff);
        cube.setPositionPID(0.1, 0.0, 0.0, 0.0);
        cube.endConfig();
//        if (Constants.getSparkConfigFromFactoryDefaults()) {
//            oldCone.restoreFactoryDefaults();
//            oldCone.setSmartCurrentLimit(20, 15, 9000);
//            oldCone.setInverted(true);
//            oldCube.restoreFactoryDefaults();
//            oldCube.setSmartCurrentLimit(20, 15, 9000);
//            //cube.setIdleMode(CANSparkMax.IdleMode.kBrake);
//            setPID(conePID, coneKp, coneKi, coneKiZone, cubeKff, 0);
//            setPID(conePID, 0.1, 0.0, 0.0, 0.0, 1);
//            setPID(cubePID, cubeKp, cubeKi, cubeKiZone, cubeKff, 0);
//            setPID(cubePID, 0.1, 0.0, 0.0, 0.0, 1);
//
//            if (Constants.getSparkBurnConfig()) {
//                oldCone.burnFlash();
//                oldCube.burnFlash();
//            }
//        }
//        disableUnusedFrames(oldCone);
//        disableUnusedFrames(oldCube);
    }

//    private void disableUnusedFrames(CANSparkMax motor) {
//        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3,500);
//        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus4,500);
//        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus5,500);
//        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus6,500);
//    }

//    private void setPID(SparkMaxPIDController pid, double kp, double ki, double kiZone, double kff, int slot) {
//        pid.setP(kp, slot);
//        pid.setI(ki, slot);
//        pid.setIZone(kiZone, slot);
//        pid.setFF(kff, slot);
//        pid.setOutputRange(-1.0, 1.0, slot);
//    }


//    public void spinConeMotor() {
//        conePID.setReference(maxSpeed, CANSparkMax.ControlType.kVelocity, 0);
//    }
//
//    public void stopConeMotor() {
//        conePID.setReference(oldCone.getEncoder().getPosition(), CANSparkMax.ControlType.kPosition, 1);
//    }
//
//
//    public void spinCubeMotor() {
//        cubePID.setReference(maxSpeed, CANSparkMax.ControlType.kVelocity, 0);
//    }
//
//    public void stopCubeMotor() {
//        cubePID.setReference(oldCube.getEncoder().getPosition(), CANSparkMax.ControlType.kPosition, 1);
//    }

    public void spinAtSpeed(double speed) {
        double clippedSpeed = Utl.clip(speed, -maxSpeed, maxSpeed);
        cone.setTargetRPM(clippedSpeed);
        cube.setTargetRPM(clippedSpeed);

//        conePID.setReference(clippedSpeed, CANSparkMax.ControlType.kVelocity, 0);
//        cubePID.setReference(clippedSpeed, CANSparkMax.ControlType.kVelocity, 0);
    }

    public void spin() {
        cone.setTargetRPM(maxSpeed);
        cube.setTargetRPM(maxSpeed);
//        conePID.setReference(maxSpeed, CANSparkMax.ControlType.kVelocity, 0);
//        cubePID.setReference(maxSpeed, CANSparkMax.ControlType.kVelocity, 0);
    }

    public void stop() {
        cone.setTargetPosition(cone.getEncoderPosition());
        cube.setTargetPosition(cube.getEncoderPosition());
//        cubePID.setReference(oldCube.getEncoder().getPosition(), CANSparkMax.ControlType.kPosition, 1);
//        conePID.setReference(oldCone.getEncoder().getPosition(), CANSparkMax.ControlType.kPosition, 1);
    }
}

