package frc.robot.commands;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class EngageIntake extends CommandBase {
    
    private Consumer<Boolean> mEngageIntake;
    private boolean mEngaged;

    public EngageIntake(Consumer<Boolean> engageIntake, boolean engaged) {
        mEngageIntake = engageIntake;
        mEngaged = engaged;
    }
    
    @Override
    public void initialize() {
        mEngageIntake.accept(mEngaged);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        mEngageIntake.accept(false);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}