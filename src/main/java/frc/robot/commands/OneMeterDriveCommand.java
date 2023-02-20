package frc.robot.commands;

import org.a05annex.frc.commands.AbsoluteTranslateCommand;

/**
 * Extend {@link AbsoluteTranslateCommand} to explicitly construct a 1m left
 * strafe translation with a no argument constructor.
 */
public class OneMeterDriveCommand extends AbsoluteTranslateCommand {

    public OneMeterDriveCommand() {
        super(0.0, 1.0);
    }
}
