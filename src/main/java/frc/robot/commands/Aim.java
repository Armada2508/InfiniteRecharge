package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionSubsystem;

public class Aim extends CommandBase {

    private VisionSubsystem mVisionSubsystem;
    
    public Aim(VisionSubsystem visionSubsystem) {
        mVisionSubsystem = visionSubsystem;
    }

    @Override
    public void initialize() {
        
    }


    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}