package frc.robot.commands;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import org.a05annex.frc.A05Constants;

public class AutoAprilCommandGroup extends SequentialCommandGroup {
    public AutoAprilCommandGroup() {
        super(new SpeedAprilTagPositionCommand(null, new A05Constants.DriverSettings("", 10), 0.806, -0.114, 1.0, 0.8, Constants.AprilTagSet.NODE),
                new CubeArmMoveCommand(new XboxController(4)));
    }
}