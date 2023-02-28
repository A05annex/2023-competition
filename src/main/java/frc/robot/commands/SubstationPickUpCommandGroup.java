package frc.robot.commands;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import org.a05annex.frc.A05Constants;

public class SubstationPickUpCommandGroup extends SequentialCommandGroup {
    public SubstationPickUpCommandGroup(XboxController altXbox, A05Constants.DriverSettings driver) {
        super(new SubstationPositionCommand(altXbox, driver), Commands.deadline(new SubstationArmMoveCommand(), new PipelineScanCommand(Constants.CLAW_CAMERA)));
    }
}