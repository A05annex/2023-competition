// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
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

    // Commands

    SampleAprilTagPositionCommand m_sampleAprilTagPositionCommand;
    PipelineScanCommand m_pipelineScanCommand;

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
            m_xboxRightStickPress = new JoystickButton(m_driveXbox, 10);

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

        m_pipelineScanCommand = new PipelineScanCommand(Constants.CLAW_CAMERA);

        m_sampleAprilTagPositionCommand = new SampleAprilTagPositionCommand(m_driveXbox, m_driver);

        m_driveSubsystem.setDefaultCommand(m_driveCommand);

        if (m_autoCommand != null) {
            m_autoCommand.setMirror(Constants.readMirrorSwitch());
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
        m_xboxStart.onTrue(new InstantCommand(m_driveSubsystem::toggleDriveMode));

        m_xboxLeftBumper.whileTrue(new FaceUpFieldCommand(m_driveXbox, m_driver));
        m_xboxRightBumper.whileTrue(new FaceDownFieldCommand(m_driveXbox, m_driver));

        // Retract and go upright with the arm when either controller's B button is pressed
        m_xboxB.onTrue(new InstantCommand(ArmSubsystem.ArmPositions.RETRACTED::goTo));
        m_altXboxB.onTrue(new InstantCommand(ArmSubsystem.ArmPositions.RETRACTED::goTo));


        // Do the Cone Place Sequence while alt Y is pressed, go to retracted when it's released
        m_altXboxY.whileTrue(new ConePlaceCommandGroup(m_altXbox, m_driver));
        m_altXboxY.onFalse(new InstantCommand(ArmSubsystem.ArmPositions.RETRACTED::goTo));

        // Do the Cube Place Sequence while alt X is pressed, go to retracted when it's released
        m_altXboxX.whileTrue(new CubePlaceCommandGroup(m_altXbox, m_driver));
        m_altXboxX.onFalse(new InstantCommand(ArmSubsystem.ArmPositions.RETRACTED::goTo));

        // Do the Substation Pickup Sequence while alt X is pressed, go to retracted when it's released
        m_altXboxA.whileTrue(new SubstationPickUpCommandGroup(m_altXbox, m_driver));
        m_altXboxA.onFalse(new InstantCommand(ArmSubsystem.ArmPositions.RETRACTED::goTo));


        // Open the claw when alt left bumper is pressed and close it when the right one is pressed.
        m_altXboxLeftBumper.onTrue(new InstantCommand(m_clawSubsystem::open));
        m_altXboxLeftBumper.onFalse(new InstantCommand(m_clawSubsystem::off)); // Turn off the solenoid when released
        m_altXboxRightBumper.onTrue(new InstantCommand(m_clawSubsystem::close));
        m_altXboxRightBumper.onFalse(new InstantCommand(m_clawSubsystem::off)); // Turn off the solenoid when released

        // Open the claw when drive A is pressed
        m_xboxA.onTrue(new InstantCommand(m_clawSubsystem::open));
        m_xboxA.onFalse(new InstantCommand(m_clawSubsystem::off)); // Turn off the solenoid when released
        // Close the claw when drive X is pressed
        m_xboxX.onTrue(new InstantCommand(m_clawSubsystem::close));
        m_xboxX.onFalse(new InstantCommand(m_clawSubsystem::off)); // Turn off the solenoid when released

        // Run the balancer while drive Y is pressed
        m_xboxY.whileTrue(new AutoBalanceCommand());

        // Toggle manual arm control when alt Back is pressed
        m_altXboxBack.toggleOnTrue(new ManualArmCommand(m_altXbox));

        // Toggle the Can Drive box on the dashboard when any of the position commands are run
        m_altXboxA.onTrue(new InstantCommand(photonSubsystem::canDriveFalse));
        m_altXboxA.onFalse(new InstantCommand(photonSubsystem::canDriveTrue));
        m_altXboxY.onTrue(new InstantCommand(photonSubsystem::canDriveFalse));
        m_altXboxY.onFalse(new InstantCommand(photonSubsystem::canDriveTrue));
        m_altXboxX.onTrue(new InstantCommand(photonSubsystem::canDriveFalse));
        m_altXboxX.onFalse(new InstantCommand(photonSubsystem::canDriveTrue));
    }
}
