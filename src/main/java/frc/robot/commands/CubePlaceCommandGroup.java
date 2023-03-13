package frc.robot.commands;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.a05annex.frc.A05Constants;

public class CubePlaceCommandGroup extends SequentialCommandGroup {
    private static XboxController altXbox;

    public CubePlaceCommandGroup(XboxController altXbox, A05Constants.DriverSettings driver) {
        super(new ConditionalCommand(new CubePositionCommand(altXbox , driver), new ConePositionCommand(altXbox, driver), CubePlaceCommandGroup::hybrid), new CubeArmMoveCommand(altXbox));

        CubePlaceCommandGroup.altXbox = altXbox;
    }

    private static boolean hybrid() {
        return !(altXbox.getPOV() == 180);
    }
}