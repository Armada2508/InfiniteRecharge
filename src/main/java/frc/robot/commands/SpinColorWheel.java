package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ColorWheelSubsystem;

public class SpinColorWheel extends CommandBase {

    private ColorWheelSubsystem mColorWheelSubsystem;
    private double mRotations;

    public SpinColorWheel(ColorWheelSubsystem colorWheelSubsystem, double rotations) {
        mColorWheelSubsystem = colorWheelSubsystem;
        mRotations = rotations;

        // Require ColorWheelSubsystem
        addRequirements(colorWheelSubsystem);
    }

    @Override
    public void initialize() {
        mColorWheelSubsystem.rotate(mRotations);
        mColorWheelSubsystem.reset();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return mColorWheelSubsystem.getRotations() < Constants.WOF.kThreshold;
    }
}