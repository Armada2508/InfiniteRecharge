package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.enums.CamMode;
import frc.robot.enums.DriveState;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class Aim extends CommandBase {

    private DriveSubsystem mDriveSubsystem;
    private VisionSubsystem mVisionSubsystem;
    private PIDController mPidController;
    private CamMode mCamMode;

    public Aim(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        mDriveSubsystem = driveSubsystem;
        mVisionSubsystem = visionSubsystem;
        mPidController = new PIDController(Constants.Vision.kPAim, Constants.Vision.kIAim, Constants.Vision.kDAim);

        mPidController.setSetpoint(Constants.Vision.kAimOffset);

        // Require DriveSubsystem and VisionSubsystem
        addRequirements(mDriveSubsystem, mVisionSubsystem);
    }

    @Override
    public void initialize() {
        mCamMode = mVisionSubsystem.getCamMode();
        mVisionSubsystem.setCamMode(CamMode.CV);
        mVisionSubsystem.setLED(true);
        mDriveSubsystem.brake();
        mDriveSubsystem.setState(DriveState.AIM);
    }


    @Override
    public void execute() {
        double offset = mVisionSubsystem.getTargetCenter().getX();
        double power = mPidController.calculate(offset);
        power = Math.abs(Math.min(Constants.Vision.kMaxAimPower, Math.abs(power))) * Math.signum(power);
        mDriveSubsystem.setPowers(-power, power);
    }

    @Override
    public void end(boolean interrupted) {
        mVisionSubsystem.setCamMode(mCamMode);
        mVisionSubsystem.setLED(false);
        mDriveSubsystem.setPowers(0, 0);
        mDriveSubsystem.setState(DriveState.DRIVE);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}