/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveClosedLoop extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem mDriveSubsystem;
  private final DoubleSupplier mThrottle;
  private final DoubleSupplier mTrim;
  private final DoubleSupplier mTurn;
  private final double mMaxPower;
  private final double mTurnRatio;
  private final double mTrimRatio;
  /**
   * Creates a new Drive.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveClosedLoop(DriveSubsystem subsystem, DoubleSupplier throttle, DoubleSupplier trim, DoubleSupplier turn, double maxPower, double turnRatio, double trimRatio) {
    mDriveSubsystem = subsystem;
    mThrottle = throttle;
    mTrim = trim;
    mTurn = turn;
    mMaxPower = maxPower;
    mTurnRatio = turnRatio;
    mTrimRatio = trimRatio;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
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

    mDriveSubsystem.driveClosedLoop((int)(powerL*Constants.kDriveFeedforward.maxAchievableVelocity(Constants.kMinBatteryVoltage, 0.0)), (int)(powerR*Constants.kDriveFeedforward.maxAchievableVelocity(Constants.kMinBatteryVoltage, 0.0)));
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
