package frc.robot.commands;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.commands.AbsoluteSmartTranslateCommand;
import org.a05annex.frc.commands.AbsoluteTranslateCommand;

public class ConePlaceCommandGroup extends SequentialCommandGroup {
    private static XboxController altXbox;

    public ConePlaceCommandGroup(XboxController altXbox, A05Constants.DriverSettings driver) {
        super(new ConePositionCommand(altXbox, driver),
                new ConditionalCommand(new AbsoluteSmartTranslateCommand(0.0, -0.50, 1.0, 7000.0).withTimeout(1.5), new AbsoluteSmartTranslateCommand(0.0, 0.57, 1.0, 7000.0).withTimeout(1.5), ConePlaceCommandGroup::getDirection),
                new AbsoluteTranslateCommand(0.65, 0.0, 0.5).withTimeout(1.5),
                new ConeArmMoveCommand(altXbox));
        ConePlaceCommandGroup.altXbox = altXbox;
    }

    private static boolean getDirection() {
        return altXbox.getPOV() == 45 || altXbox.getPOV() == 90 || altXbox.getPOV() == 135;
    }
}