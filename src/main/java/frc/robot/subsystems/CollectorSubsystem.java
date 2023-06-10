package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.a05annex.frc.subsystems.SparkNeo;
import org.a05annex.frc.subsystems.SparkNeo550;
import org.a05annex.util.Utl;

public class CollectorSubsystem extends SubsystemBase {

    private final SparkNeo550 cone = SparkNeo550.factory(Constants.CAN_Devices.CONE_MOTOR);
    private final double coneKp = 0.0001, coneKi = 0.0, coneKiZone = 0.0;

    private final SparkNeo550 cube = SparkNeo550.factory(Constants.CAN_Devices.CUBE_MOTOR);
    private final double cubeKp = 0.0001, cubeKi = 0.0, cubeKiZone = 0.0, cubeKff = 0.0;

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
    }

    public void spinAtSpeed(double speed) {
        double clippedSpeed = Utl.clip(speed, -maxSpeed, maxSpeed);
        cone.setTargetRPM(clippedSpeed);
        cube.setTargetRPM(clippedSpeed);
    }

    public void spin() {
        cone.setTargetRPM(maxSpeed);
        cube.setTargetRPM(maxSpeed);
    }

    public void stop() {
        cone.setTargetPosition(cone.getEncoderPosition());
        cube.setTargetPosition(cube.getEncoderPosition());
    }
}

