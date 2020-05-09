/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.lib.drive.BasicDrive;
import frc.lib.motion.DifferentialDriveWheelPowers;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveClosedLoop extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem mDriveSubsystem;
  private final DoubleSupplier mThrottle;
  private final DoubleSupplier mTrim;
  private final DoubleSupplier mTurn;
  private BasicDrive mBasicDrive;
  /**
   * Creates a new DriveClosedLoop Command.
   *
   */
  public DriveClosedLoop(DriveSubsystem driveSubsystem, DoubleSupplier throttle, DoubleSupplier trim, DoubleSupplier turn) {
    mDriveSubsystem = driveSubsystem;
    mThrottle = throttle;
    mTrim = trim;
    mTurn = turn;
    double maxPower = Constants.Drive.kMaxPower;
    double turnRatio = Constants.Drive.kTurnRatio;
    double trimRatio = Constants.Drive.kTrimRatio;
    double deadband = Constants.Drive.kDeadbandThreshold;

    mBasicDrive = new BasicDrive();
    mBasicDrive.config(turnRatio, trimRatio, maxPower, deadband);

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

    DifferentialDriveWheelPowers powers = mBasicDrive.drive(mThrottle.getAsDouble(), mTurn.getAsDouble(), mTrim.getAsDouble());

    DifferentialDriveWheelSpeeds speeds = new DifferentialDriveWheelSpeeds(
      powers.getLeft() * Constants.Drive.kFeedforward.maxAchievableVelocity(Constants.Robot.kMinBatteryVoltage, 0.0),
      powers.getRight() * Constants.Drive.kFeedforward.maxAchievableVelocity(Constants.Robot.kMinBatteryVoltage, 0.0));

    mDriveSubsystem.setVelocity(speeds);
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
