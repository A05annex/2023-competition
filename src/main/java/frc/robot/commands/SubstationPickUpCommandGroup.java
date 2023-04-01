package frc.robot.commands;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import org.a05annex.frc.A05Constants;

public class SubstationPickUpCommandGroup extends SequentialCommandGroup {
    public SubstationPickUpCommandGroup(XboxController driveXbox, XboxController altXbox, A05Constants.DriverSettings driver) {
        super(new SpeedAprilTagPositionCommand(driveXbox, driver, 0.8382, 0.6484, 1.0, 0.8, Constants.AprilTagSet.SUBSTATION), new SubstationArmMoveCommand());
    }
    //X=0.8382
    //Y=0.6484
}