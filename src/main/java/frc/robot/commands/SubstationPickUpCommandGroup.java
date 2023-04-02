package frc.robot.commands;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import org.a05annex.frc.A05Constants;

public class SubstationPickUpCommandGroup extends SequentialCommandGroup {
    private static XboxController altXbox;

    public SubstationPickUpCommandGroup(XboxController driveXbox, XboxController altXbox, A05Constants.DriverSettings driver) {
        super(new ConditionalCommand(
                new SequentialCommandGroup(new SpeedAprilTagPositionCommand(driveXbox, driver, 0.84, 0.6484, 1.0, 0.8, Constants.AprilTagSet.SUBSTATION),
                        new SubstationCubeCommand()),
                new SequentialCommandGroup(new SpeedAprilTagPositionCommand(driveXbox, driver, 0.69, 0.6484, 1.0, 0.8, Constants.AprilTagSet.SUBSTATION),
                        new SubstationConeCommand()),
                SubstationPickUpCommandGroup::isCube),
                new InstantCommand(ArmSubsystem.ArmPositions.RETRACTED::goTo));

        SubstationPickUpCommandGroup.altXbox = altXbox;
    }
    //X=0.8382
    //Y=0.6484

    private static boolean isCube() {
        return altXbox.getPOV() == 0 || altXbox.getPOV() == 45 || altXbox.getPOV() == 315;
    }
}