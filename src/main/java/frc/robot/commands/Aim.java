package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class Aim extends CommandBase {

    private DriveSubsystem mDriveSubsystem;
    private VisionSubsystem mVisionSubsystem;
    private PIDController mPidController;

    public Aim(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        mDriveSubsystem = driveSubsystem;
        mVisionSubsystem = visionSubsystem;
        mPidController = new PIDController(Constants.Vision.kPAim, Constants.Vision.kIAim, Constants.Vision.kDAim);

        mPidController.setSetpoint(0.0);

        // Require DriveSubsystem and VisionSubsystem
        addRequirements(mDriveSubsystem, mVisionSubsystem);
    }

    @Override
    public void initialize() {
        mVisionSubsystem.setLED(true);
    }


    @Override
    public void execute() {
        double offset = mVisionSubsystem.getTargetCenter().getX();
        double power = mPidController.calculate(offset);
        power = Math.min(Constants.Drive.kMaxAimPower, power);
        mDriveSubsystem.setPowers(-power, power);
        System.out.println(power);
    }

    @Override
    public void end(boolean interrupted) {
        mVisionSubsystem.setLED(false);
        mDriveSubsystem.setPowers(0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}