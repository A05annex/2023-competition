package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.SpeedCachedSwerve;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.commands.A05DriveCommand;
import org.a05annex.util.AngleD;
import org.a05annex.util.Utl;


public class FaceDownFieldCommand extends A05DriveCommand {
    public FaceDownFieldCommand(XboxController xbox, A05Constants.DriverSettings driver) {
        // NOTE: the super adds the drive subsystem requirement
        super(SpeedCachedSwerve.getInstance(), xbox, driver);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        conditionStick();
        navX.setExpectedHeading(navX.getHeadingInfo().getClosestDownField());
        conditionedRotate = new AngleD(navX.getHeadingInfo().expectedHeading).subtract(new AngleD(navX.getHeadingInfo().heading))
                .getRadians() * A05Constants.getDriveOrientationkp();
        conditionedRotate = Utl.clip(conditionedRotate, -0.5, 0.5);
        iSwerveDrive.swerveDrive(conditionedDirection, conditionedSpeed, conditionedRotate);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
