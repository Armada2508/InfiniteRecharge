package frc.robot.commands;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinRoller extends CommandBase {

    private ShooterSubsystem mShooterSubsystem;
    private SlewRateLimiter mLimiter;
    private double mRpm;

    public SpinRoller(ShooterSubsystem shooterSubsystem, double rpm, int maxSlewRate) {
        mShooterSubsystem = shooterSubsystem;
        mRpm = rpm;
        mLimiter = new SlewRateLimiter(maxSlewRate);

        // Require ShooterSubsystem
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        mShooterSubsystem.spin(mLimiter.calculate(mShooterSubsystem.getRPM()));
    }

    @Override
    public void execute() {
        mShooterSubsystem.spin(mLimiter.calculate(mRpm));
    }

    @Override
    public void end(boolean interrupted) {
        mShooterSubsystem.coast();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}