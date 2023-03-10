// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.A05Robot;

import java.util.Collections;


/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends A05Robot
{
    
    /**
     * This method is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit()
    {
        // Set the drive constants that are specific to this swerve geometry.
        // Some drive geometry is passed in RobotContainer's constructor
        Constants.setDriveOrientationkp(Constants.DRIVE_ORIENTATION_kP);
        Constants.setPrintDebug(false);

        // Load the robot settings list
        Collections.addAll(A05Constants.ROBOT_SETTINGS_LIST,Constants.ROBOT_SETTINGS);
        // Load the autonomous path list
        Collections.addAll(A05Constants.AUTONOMOUS_PATH_LIST,Constants.AUTONOMOUS_PATHS);
        // Load the driver list
        Collections.addAll(A05Constants.DRIVER_SETTINGS_LIST,Constants.DRIVER_SETTINGS);


        SmartDashboard.putData("Cone High", new InstantCommand(ArmSubsystem.ArmPositions.CONE_HIGH::goTo));
        SmartDashboard.putData("Cone Middle", new InstantCommand(ArmSubsystem.ArmPositions.CONE_MEDIUM::goTo));
        SmartDashboard.putData("Cube High", new InstantCommand(ArmSubsystem.ArmPositions.CUBE_HIGH::goTo));
        SmartDashboard.putData("Cube Middle", new InstantCommand(ArmSubsystem.ArmPositions.CUBE_MEDIUM::goTo));
        SmartDashboard.putData("Hybrid", new InstantCommand(ArmSubsystem.ArmPositions.HYBRID::goTo));
        SmartDashboard.putData("Retracted", new InstantCommand(ArmSubsystem.ArmPositions.RETRACTED::goTo));

        Constants.updateConstant("angle", 0.0);
        Constants.updateConstant("speed", 0.0);
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        setRobotContainer(new RobotContainer());
    }
    

    
    /** This method is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        ArmSubsystem.getInstance().stopAllMotors();
    }

    
    @Override
    public void disabledPeriodic() {
        A05Constants.printIDs();
    }

    
    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit()
    {
        // Sets up autonomous command
        super.autonomousInit();
    }
    
    
    /** This method is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}
    
    
    @Override
    public void teleopInit()
    {
        // Cancels autonomous command
        super.teleopInit();

        ArmSubsystem.getInstance().setExtensionPosition(ArmSubsystem.getInstance().getExtensionPosition());
        ArmSubsystem.getInstance().setPivotPosition(ArmSubsystem.getInstance().getPivotPosition());
    }
    
    
    /** This method is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        super.teleopPeriodic();
        A05Constants.printIDs();
    }
    
    @Override
    public void testInit()
    {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }
    
    
    /** This method is called periodically during test mode. */
    @Override
    public void testPeriodic() {}
}
