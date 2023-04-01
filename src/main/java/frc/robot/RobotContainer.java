// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import org.a05annex.frc.A05RobotContainer;

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
    ClawSubsystem m_clawSubsystem = ClawSubsystem.getInstance();
    PhotonVisionSubsystem photonSubsystem = PhotonVisionSubsystem.getInstance();
    SpeedCachedSwerve speedCachedSwerve = SpeedCachedSwerve.getInstance();
    CollectorSubsystem collectorSubsystem = CollectorSubsystem.getInstance();

    // Commands

    XboxController m_altXbox = new XboxController(Constants.ALT_XBOX_PORT);

    // controller button declarations
    @SuppressWarnings("unused")
    JoystickButton
            m_xboxA = new JoystickButton(m_driveXbox, 1),
            m_altXboxA = new JoystickButton(m_altXbox, 1),
            m_xboxB = new JoystickButton(m_driveXbox, 2),
            m_altXboxB = new JoystickButton(m_altXbox, 2),
            m_xboxX = new JoystickButton(m_driveXbox, 3),
            m_altXboxX = new JoystickButton(m_altXbox, 3),
            m_xboxY = new JoystickButton(m_driveXbox, 4),
            m_altXboxY = new JoystickButton(m_altXbox, 4),
            m_xboxLeftBumper = new JoystickButton(m_driveXbox, 5),
            m_altXboxLeftBumper = new JoystickButton(m_altXbox, 5),
            m_xboxRightBumper = new JoystickButton(m_driveXbox, 6),
            m_altXboxRightBumper = new JoystickButton(m_altXbox, 6),
            m_xboxBack = new JoystickButton(m_driveXbox, 7),
            m_altXboxBack = new JoystickButton(m_altXbox, 7),
            m_xboxStart = new JoystickButton(m_driveXbox, 8),
            m_altXboxStart = new JoystickButton(m_altXbox, 8),
            m_xboxLeftStickPress = new JoystickButton(m_driveXbox, 9),
            m_altXboxLeftStickPress = new JoystickButton(m_altXbox, 9),
            m_xboxRightStickPress = new JoystickButton(m_driveXbox, 10),
            m_altXboxRightStickPress = new JoystickButton(m_altXbox, 10);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        super();
        // finish swerve drive initialization for this specific robt.
        m_navx.setYawCalibrationFactor(m_robotSettings.m_navxYawCalibration);
        speedCachedSwerve.setDriveSubsystem(m_driveSubsystem);
        speedCachedSwerve.setCacheLength(400);
        speedCachedSwerve.setDriveGeometry(m_robotSettings.m_length, m_robotSettings.m_width,
                m_robotSettings.m_rf, m_robotSettings.m_rr,
                m_robotSettings.m_lf, m_robotSettings.m_lr,
                m_robotSettings.m_maxSpeedCalibration);

        m_driveCommand = new DriveCommand(m_driveXbox, m_driver);

        m_driveSubsystem.setDefaultCommand(m_driveCommand);
        ArmSubsystem.getInstance().setDefaultCommand(new ManualArmCommand(m_altXbox));

        if (m_autoCommand != null) {
            m_autoCommand.setMirror(!Constants.readMirrorSwitch()); // Something was backwards
        }

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

        // Reset field relative to current robot heading when drive Back is pressed
        m_xboxBack.onTrue(new InstantCommand(m_navx::initializeHeadingAndNav));

        // Toggle between robot and field relative when drive Start is pressed
        m_xboxStart.onTrue(new InstantCommand(speedCachedSwerve::toggleDriveMode));

        m_xboxA.whileTrue(new FaceUpFieldCommand(m_driveXbox, m_driver));
        m_xboxY.whileTrue(new FaceDownFieldCommand(m_driveXbox, m_driver));

        // Retract and go upright with the arm when either controller's B button is pressed
        m_xboxB.onTrue(new InstantCommand(ArmSubsystem.ArmPositions.RETRACTED::goTo));
        m_altXboxB.onTrue(new InstantCommand(ArmSubsystem.ArmPositions.RETRACTED::goTo));


        // Do the Cone Place Sequence while alt Y is pressed, go to retracted when it's released
        m_altXboxY.whileTrue(new ConePlaceCommandGroup(m_altXbox, m_driveXbox, m_driver));
        m_altXboxY.onFalse(new InstantCommand(ArmSubsystem.ArmPositions.RETRACTED::goTo));

        // Do the Cube Place Sequence while alt X is pressed, go to retracted when it's released
        m_altXboxX.whileTrue(new CubePlaceCommandGroup(m_altXbox, m_driveXbox, m_driver));
        m_altXboxX.onFalse(new InstantCommand(ArmSubsystem.ArmPositions.RETRACTED::goTo));

        // Do the Substation Pickup Sequence while alt X is pressed, go to retracted when it's released
        m_altXboxA.whileTrue(new SubstationPickUpCommandGroup(m_driveXbox, m_altXbox, m_driver));
        m_altXboxA.onFalse(new InstantCommand(ArmSubsystem.ArmPositions.RETRACTED::goTo));

        m_xboxLeftBumper.onTrue(new CollectorEjectCommand());
        m_xboxRightBumper.whileTrue(new InstantCommand(collectorSubsystem::spin)).whileFalse(new InstantCommand(collectorSubsystem::stop));
        m_altXboxLeftBumper.onTrue(new CollectorEjectCommand());
        m_altXboxRightBumper.whileTrue(new ConditionalCommand(new InstantCommand(collectorSubsystem::spin), new GroundPickupCommand(), ArmSubsystem.getInstance()::isManualControl))
                .whileFalse(new InstantCommand(collectorSubsystem::stop));


        /*
        // Open the claw when alt left bumper is pressed and close it when the right one is pressed.
        m_altXboxLeftBumper.onTrue(new InstantCommand(m_clawSubsystem::open));
        m_altXboxLeftBumper.onFalse(new InstantCommand(m_clawSubsystem::off)); // Turn off the solenoid when released
        m_altXboxRightBumper.onTrue(new InstantCommand(m_clawSubsystem::close));
        m_altXboxRightBumper.onFalse(new InstantCommand(m_clawSubsystem::off)); // Turn off the solenoid when released

        m_xboxLeftBumper.onTrue(new InstantCommand(m_clawSubsystem::open));
        m_xboxLeftBumper.onFalse(new InstantCommand(m_clawSubsystem::off)); // Turn off the solenoid when released
        m_xboxRightBumper.onTrue(new InstantCommand(m_clawSubsystem::close));
        m_xboxRightBumper.onFalse(new InstantCommand(m_clawSubsystem::off)); // Turn off the solenoid when released
        */

        // Run the balancer while drive X is pressed
        m_xboxX.whileTrue(new AutoBalanceCommand());

        // Toggle manual arm control when alt Back is pressed
        m_altXboxBack.onTrue(new InstantCommand(ArmSubsystem.getInstance()::toggleManualControl));

        // Go to the substaion positions
        m_altXboxLeftStickPress.onTrue(new InstantCommand(ArmSubsystem.ArmPositions.SUBSTATION_CUBE::goTo));
        m_altXboxRightStickPress.onTrue(new InstantCommand(ArmSubsystem.ArmPositions.SUBSTATION_CONE::goTo));
    }
}
