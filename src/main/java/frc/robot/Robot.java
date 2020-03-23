/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
  private RobotContainer mRobotContainer;
  private Command mAutonomousCommand;


  @Override
  public void robotInit() {
    // Make a new RobotContainer to contain all the stuff we want to do
    mRobotContainer = new RobotContainer();

    // Initialization
    mRobotContainer.robotInit();
    
    
  }

  @Override
  public void robotPeriodic() {
    // Run the scheduler to update commands
    CommandScheduler.getInstance().run();

    // Update the dashboard
    mRobotContainer.updateDashboard();

    // Update the dashboard(again)
    mRobotContainer.updateLogger();

    // Update the Shooter RPM
    mRobotContainer.updateRPM();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    // Cancel all running commands
    CommandScheduler.getInstance().cancelAll();
    // We have changed modes
    mRobotContainer.changeMode();

    // If the match just ended, stop logging data
    if(isOperatorControl()) {
      mRobotContainer.stopDashboardCapture();
    }
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    // We have changed modes
    mRobotContainer.changeMode();
    // Start logging data if we're connected to the FMS
    mRobotContainer.startDashboardCapture();
    // Get a command group to run in autonomous
    mAutonomousCommand = mRobotContainer.getAutonomousCommand();

    // If we got an autonomous command
    if (mAutonomousCommand != null) {
      // Run the autonomous command
      mAutonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    mRobotContainer.changeMode();
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancel all commands
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

}
