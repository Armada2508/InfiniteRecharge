package frc.robot.commands;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinRoller extends CommandBase {

    private ShooterSubsystem mShooterSubsystem;
    private SlewRateLimiter mLimiter;
    private double mRpm;
    private boolean mAtSpeed;
    private boolean mCoast;

    public SpinRoller(ShooterSubsystem shooterSubsystem, double rpm) {
        mCoast = rpm == 0;

        mShooterSubsystem = shooterSubsystem;
        mRpm = rpm;
        mLimiter = new SlewRateLimiter(Constants.Shooter.kMaxShooterSlewRate);
        mAtSpeed = false;

        mShooterSubsystem.setCurrent(Constants.Shooter.kShooterConfig.getContinuousCurrent());

        // Require ShooterSubsystem
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        mShooterSubsystem.spin(mLimiter.calculate(mShooterSubsystem.getRPM()));
    }

    @Override
    public void execute() {
        if(Math.abs(mShooterSubsystem.getRPM() - mRpm) < Constants.Shooter.kStableRPMThreshold) {
            mAtSpeed = true;
        }
        if(mAtSpeed) {
            mShooterSubsystem.setCurrent(Constants.Shooter.kShooterStableCurrentLimit);
        }
        mShooterSubsystem.spin(mLimiter.calculate(mRpm));
    }

    @Override
    public void end(boolean interrupted) {
        mShooterSubsystem.coast();
    }

    @Override
    public boolean isFinished() {
        return mCoast;
    }
}