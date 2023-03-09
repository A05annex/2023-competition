// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArmStickPositionCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import org.a05annex.frc.A05RobotContainer;
import org.a05annex.frc.commands.AbsoluteSmartTranslateCommand;
import org.a05annex.frc.commands.AbsoluteTranslateCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer extends A05RobotContainer
{
    // The robot's subsystems and commands are defined here...
    // NavX, DriveSubsystem, DriveXbox have already been made in A05RobotContainer

    // Subsystems
    ArmSubsystem m_armSubsystem = ArmSubsystem.getInstance();
    ClawSubsystem m_clawSubsystem = ClawSubsystem.getInstance();

    // Commands
    Command m_armStickCommand;

    //TODO: Uncomment if you have alternate xbox controller, you need to uncomment a constant too
    XboxController m_altXbox = new XboxController(Constants.ALT_XBOX_PORT);

    // controller button declarations
    JoystickButton m_xboxA = new JoystickButton(m_driveXbox, 1);
    JoystickButton m_altXboxA = new JoystickButton(m_altXbox, 1);
    JoystickButton m_xboxB = new JoystickButton(m_driveXbox, 2);
    JoystickButton m_altXboxB = new JoystickButton(m_altXbox, 2);
    JoystickButton m_altXboxX = new JoystickButton(m_altXbox, 3);
    JoystickButton m_altXboxY = new JoystickButton(m_altXbox, 4);
    JoystickButton m_xboxX = new JoystickButton(m_driveXbox, 3);
    JoystickButton m_xboxY = new JoystickButton(m_driveXbox, 4);
    JoystickButton m_xboxLeftBumper = new JoystickButton(m_driveXbox, 5);
    JoystickButton m_xboxRightBumper = new JoystickButton(m_driveXbox, 6);
    JoystickButton m_xboxBack = new JoystickButton(m_driveXbox, 7);
    JoystickButton m_xboxStart = new JoystickButton(m_driveXbox, 8);
    JoystickButton m_xboxLeftStickPress = new JoystickButton(m_driveXbox, 9);
    JoystickButton m_xboxRightStickPress = new JoystickButton(m_driveXbox, 10);

    // alt xbox controller buttons
    //JoystickButton m_altXboxA = new JoystickButton(m_altXbox, 1);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        super();
        // finish swerve drive initialization for this specific robt.
        m_driveSubsystem.setDriveGeometry(m_robotSettings.m_length, m_robotSettings.m_width,
                m_robotSettings.m_rf, m_robotSettings.m_rr,
                m_robotSettings.m_lf, m_robotSettings.m_lr,
                m_robotSettings.m_maxSpeedCalibration);

        m_driveCommand = new DriveCommand(m_driveXbox, m_driver);
        m_armStickCommand = new ArmStickPositionCommand(m_altXbox);

        m_driveSubsystem.setDefaultCommand(m_driveCommand);
        m_armSubsystem.setDefaultCommand(m_armStickCommand);

        //TODO: add auto

        // Configure the button bindings
        configureButtonBindings();
    }


    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings()
    {
        // Add button to command mappings here.
        // See https://docs.wpilib.org/en/stable/docs/software/commandbased/binding-commands-to-triggers.html

        m_xboxBack.onTrue(new InstantCommand(m_navx::initializeHeadingAndNav)); // Reset the NavX field relativity
        m_altXboxA.whileTrue(new InstantCommand(m_armSubsystem::goToCalcPos));
        m_altXboxB.whileTrue(new InstantCommand(m_armSubsystem::stopAllMotors));
        m_altXboxX.onTrue(new AbsoluteTranslateCommand(0.0, 1.0));
        m_altXboxY.onTrue(new AbsoluteSmartTranslateCommand(0.0, 1.0, 1.0, 10000.0));
        m_xboxB.onTrue(new InstantCommand(m_clawSubsystem::close));
        m_xboxA.onTrue(new InstantCommand(m_clawSubsystem::open));
        m_xboxX.onTrue(new InstantCommand(m_clawSubsystem::bleed));
    }
}
