package frc.robot.commands;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.a05annex.frc.A05Constants;

public class CubePlaceCommandGroup extends SequentialCommandGroup {

    public CubePlaceCommandGroup(XboxController altXbox, A05Constants.DriverSettings driver) {
        super(new CubePositionCommand(altXbox , driver), new CubeArmMoveCommand(altXbox));
    }

}