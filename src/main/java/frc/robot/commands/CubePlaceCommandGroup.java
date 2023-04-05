package frc.robot.commands;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import org.a05annex.frc.A05Constants;

public class CubePlaceCommandGroup extends SequentialCommandGroup {
    private static XboxController altXbox;

    public CubePlaceCommandGroup(XboxController altXbox, XboxController driveXbox, A05Constants.DriverSettings driver) {
        super(new ConditionalCommand(new SpeedAprilTagPositionCommand(driveXbox, driver,
                0.85, -0.114, 1.0, 0.8, Constants.AprilTagSet.NODE),
                new SpeedAprilTagPositionCommand(driveXbox, driver,
                        1.0, -0.11, 1.0, 0.8, Constants.AprilTagSet.NODE), CubePlaceCommandGroup::hybrid),
                new CubeArmMoveCommand(altXbox));

        CubePlaceCommandGroup.altXbox = altXbox;
    }

    private static boolean hybrid() {
        return !(altXbox.getPOV() == 180);
    }
}