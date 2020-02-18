package frc.robot.commands;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinRoller extends CommandBase {

    private ShooterSubsystem m_shooterSubsystem;
    private SlewRateLimiter m_limiter;
    private double m_rpm;

    public SpinRoller(ShooterSubsystem shooterSubsystem, double rpm, int maxSlewRate) {
        m_shooterSubsystem = shooterSubsystem;
        m_rpm = rpm;
        m_limiter = new SlewRateLimiter(maxSlewRate);
    }

    @Override
    public void initialize() {
        m_shooterSubsystem.spin(m_limiter.calculate(m_shooterSubsystem.getRPM()));
    }

    @Override
    public void execute() {
        m_shooterSubsystem.spin(m_limiter.calculate(m_rpm));
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.spin(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}