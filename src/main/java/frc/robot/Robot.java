// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
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
        Constants.setPrintDebug(true);

        // Load the robot settings list
        Collections.addAll(A05Constants.ROBOT_SETTINGS_LIST,Constants.ROBOT_SETTINGS);
        // Load the autonomous path list
        Collections.addAll(A05Constants.AUTONOMOUS_PATH_LIST,Constants.AUTONOMOUS_PATHS);
        // Load the driver list
        Collections.addAll(A05Constants.DRIVER_SETTINGS_LIST,Constants.DRIVER_SETTINGS);

        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        setRobotContainer(new RobotContainer());
    }
    

    
    /** This method is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}
    
    
    @Override
    public void disabledPeriodic() {
        SmartDashboard.putNumber("Pivot Position", ArmSubsystem.getInstance().getPivotPosition());
        SmartDashboard.putNumber("Extension Position", ArmSubsystem.getInstance().getExtensionPosition());
        SmartDashboard.putNumber("Ext. Calc. Pos.", ArmSubsystem.getInstance().pivotToExtension());
        A05Constants.printIDs();
        SmartDashboard.putNumber("Claw Position", ClawSubsystem.getInstance().getPosition());
    }
    
    
    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit()
    {
        // Sets up autonomous command
        super.autonomousInit();
        //TODO: Add other things here (setting limelight pipeline)
    }
    
    
    /** This method is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}
    
    
    @Override
    public void teleopInit()
    {
        // Cancels autonomous command
        super.teleopInit();
        //TODO: Can add other things here

        ArmSubsystem.getInstance().setExtensionPosition(ArmSubsystem.getInstance().getExtensionPosition());
        ArmSubsystem.getInstance().setPivotPosition(ArmSubsystem.getInstance().getPivotPosition());
    }
    
    
    /** This method is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        super.teleopPeriodic();
        A05Constants.printIDs();
        SmartDashboard.putNumber("Pivot Position", ArmSubsystem.getInstance().getPivotPosition());
        SmartDashboard.putNumber("Extension Position", ArmSubsystem.getInstance().getExtensionPosition());
        SmartDashboard.putNumber("Calc. Pos.", ArmSubsystem.getInstance().pivotToExtension());
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
