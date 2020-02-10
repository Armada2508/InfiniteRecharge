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
  private final DriveSubsystem m_driveSubsystem;
  private final VisionSubsystem m_visionSubsystem;
  private final double m_kTurn;
  private final double m_kThrottle;
  private final double m_maxOutput;
  private final double m_targetWidth;
  private final double m_targetDistance;

  /**
   * Creates a new FollowTarget.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FollowTarget(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, double kTurn, double kThrottle, double maxOutput, double targetWidth, double targetDistance, FOV fov, Resolution res) {
    m_driveSubsystem = driveSubsystem;
    m_visionSubsystem = visionSubsystem;
    m_kTurn = kTurn;
    m_kThrottle = kThrottle;
    m_maxOutput = maxOutput;
    m_targetWidth = targetWidth;
    m_targetDistance = targetDistance;


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
    addRequirements(visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_visionSubsystem.targetFound()) {
      double x = m_visionSubsystem.getX();
      double distance = m_visionSubsystem.getDistanceWidth(m_targetWidth);
      double throttle = (distance - m_targetDistance) * m_kThrottle;
      double turn = x * m_kTurn;
      if(Math.abs(throttle) > m_maxOutput) {
        throttle = m_maxOutput * Math.signum(throttle);
      }
      if(Math.abs(turn) > m_maxOutput) {
        turn = m_maxOutput * Math.signum(turn);
      }
      m_driveSubsystem.setArcade(throttle, turn);
    } else {
      m_driveSubsystem.setArcade(0, 0);
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
