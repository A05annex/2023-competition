package frc.robot.commands;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.commands.AbsoluteSmartTranslateCommand;

public class SubstationPickUpCommandGroup extends SequentialCommandGroup {
    public SubstationPickUpCommandGroup(XboxController altXbox, A05Constants.DriverSettings driver) {
        super(new SubstationPositionCommand(altXbox, driver), new AbsoluteSmartTranslateCommand(0.05, 0.18, 0.2, 4000.0, false), new SubstationArmMoveCommand());
    }
}