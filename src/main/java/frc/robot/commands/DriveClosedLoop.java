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
  private final DriveSubsystem m_driveSubsystem;
  private final DoubleSupplier m_throttle;
  private final DoubleSupplier m_trim;
  private final DoubleSupplier m_turn;
  private final double m_maxPower;
  private final double m_turnRatio;
  private final double m_trimRatio;
  /**
   * Creates a new Drive.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveClosedLoop(DriveSubsystem subsystem, DoubleSupplier throttle, DoubleSupplier trim, DoubleSupplier turn, double maxPower, double turnRatio, double trimRatio) {
    m_driveSubsystem = subsystem;
    m_throttle = throttle;
    m_trim = trim;
    m_turn = turn;
    m_maxPower = maxPower;
    m_turnRatio = turnRatio;
    m_trimRatio = trimRatio;
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

    double throttle = m_throttle.getAsDouble();
    double trim = m_trim.getAsDouble();
    double turn = m_turn.getAsDouble();
    
    turn *= m_turnRatio;
    trim *= m_trimRatio;
    turn += trim; 

    double powerL = throttle + turn;
    double powerR = throttle - turn;

    powerR *= m_maxPower;
    powerL *= m_maxPower;

    double turningPower = powerL - powerR;
    if(turningPower > 0 && powerL > m_maxPower) {
      powerL = m_maxPower;
      powerR = m_maxPower - turningPower;
    } else if(turningPower < 0 && powerR > m_maxPower) {
      powerR = m_maxPower;
      powerL = m_maxPower + turningPower;
    }

    m_driveSubsystem.driveClosedLoop((int)(powerL*Constants.kDriveFeedforward.maxAchievableVelocity(Constants.kMinBatteryVoltage, 0.0)), (int)(powerR*Constants.kDriveFeedforward.maxAchievableVelocity(Constants.kMinBatteryVoltage, 0.0)));
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
