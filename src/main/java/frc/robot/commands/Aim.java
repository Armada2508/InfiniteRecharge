package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class Aim extends CommandBase {

    private DriveSubsystem mDriveSubsystem;
    private VisionSubsystem mVisionSubsystem;
    
    public Aim(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        mDriveSubsystem = driveSubsystem;
        mVisionSubsystem = visionSubsystem;

        // Require DriveSubsystem and VisionSubsystem
        addRequirements(mDriveSubsystem, mVisionSubsystem);
    }

    @Override
    public void initialize() {
        
    }


    @Override
    public void execute() {
        double offset = mVisionSubsystem.getTargetCenter().getX();
        mDriveSubsystem.setPowers(offset*Constants.Vision.kPAim, -offset*Constants.Vision.kPAim);
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}