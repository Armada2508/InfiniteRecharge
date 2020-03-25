/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.lib.vision.*;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class FollowTarget extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem mDriveSubsystem;
  private final VisionSubsystem mVisionSubsystem;
  private final double mTurn;
  private final double mThrottle;
  private final double mMaxOutput;
  private final double mTargetWidth;
  private final double mTargetDistance;

  /**
   * Creates a new FollowTarget.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FollowTarget(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, double turn, double throttle, double maxOutput, double targetWidth, double targetDistance, FOV fov, Resolution res) {
    mDriveSubsystem = driveSubsystem;
    mVisionSubsystem = visionSubsystem;
    mTurn = turn;
    mThrottle = throttle;
    mMaxOutput = maxOutput;
    mTargetWidth = targetWidth;
    mTargetDistance = targetDistance;


    // Reqire DriveSubsystem and VisionSubsystem
    addRequirements(driveSubsystem, visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(mVisionSubsystem.targetFound()) {
      double x = mVisionSubsystem.getX();
      double distance = mVisionSubsystem.getDistanceWidth(mTargetWidth);
      double throttle = (distance - mTargetDistance) * mThrottle;
      double turn = x * mTurn;
      if(Math.abs(throttle) > mMaxOutput) {
        throttle = mMaxOutput * Math.signum(throttle);
      }
      if(Math.abs(turn) > mMaxOutput) {
        turn = mMaxOutput * Math.signum(turn);
      }
      mDriveSubsystem.setArcade(throttle, turn);
    } else {
      mDriveSubsystem.setArcade(0, 0);
    }
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
