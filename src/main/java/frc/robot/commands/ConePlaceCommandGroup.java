package frc.robot.commands;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.commands.AbsoluteTranslateCommand;

public class ConePlaceCommandGroup extends SequentialCommandGroup {
    private static XboxController altXbox;

    public ConePlaceCommandGroup(XboxController altXbox, A05Constants.DriverSettings driver) {
        super(new PlacePositionCommand(altXbox, driver),
                new ConditionalCommand(new AbsoluteTranslateCommand(0.0, -0.61), new AbsoluteTranslateCommand(0.0, 0.57), ConePlaceCommandGroup::direction),
                new AbsoluteTranslateCommand(0.70, 0.0).withTimeout(1.5),
                new ConeArmMoveCommand(altXbox));
        SmartDashboard.putNumber("pov", altXbox.getPOV());
        ConePlaceCommandGroup.altXbox = altXbox;
    }

    private static boolean direction() {
        return altXbox.getPOV() == 45 || altXbox.getPOV() == 90 || altXbox.getPOV() == 135;
    }
}