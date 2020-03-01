/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.lib.drive.JoystickUtil;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Drive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem mDriveSubsystem;
  private DoubleSupplier mThrottle;
  private DoubleSupplier mTrim;
  private DoubleSupplier mTurn;
  private double mMaxPower;
  private double mTurnRatio;
  private double mTrimRatio;
  /**
   * Creates a new Drive.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Drive(DriveSubsystem driveSubsystem, DoubleSupplier throttle, DoubleSupplier trim, DoubleSupplier turn) {
    mDriveSubsystem = driveSubsystem;
    mThrottle = throttle;
    mTrim = trim;
    mTurn = turn;
    mMaxPower = Constants.Drive.kMaxPower;
    mTurnRatio = Constants.Drive.kTurnRatio;
    mTrimRatio = Constants.Drive.kTrimRatio;

    // Require DriveSubsystem
    addRequirements(mDriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double throttle = mThrottle.getAsDouble();
    double trim = mTrim.getAsDouble();
    double turn = mTurn.getAsDouble();

    throttle = JoystickUtil.deadband(throttle, Constants.Drive.kDeadbandThreshold);
    trim = JoystickUtil.deadband(trim, Constants.Drive.kDeadbandThreshold);
    turn = JoystickUtil.deadband(turn, Constants.Drive.kDeadbandThreshold);
    
    turn *= mTurnRatio;
    trim *= mTrimRatio;
    turn += trim; 

    double powerL = throttle + turn;
    double powerR = throttle - turn;

    powerR *= mMaxPower;
    powerL *= mMaxPower;

    double turningPower = powerL - powerR;
    if(turningPower > 0 && powerL > mMaxPower) {
      powerL = mMaxPower;
      powerR = mMaxPower - turningPower;
    } else if(turningPower < 0 && powerR > mMaxPower) {
      powerR = mMaxPower;
      powerL = mMaxPower + turningPower;
    }

    mDriveSubsystem.setPowers(powerL, powerR);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    return;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
