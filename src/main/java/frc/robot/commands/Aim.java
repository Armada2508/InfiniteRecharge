package frc.robot.commands;

import badlog.lib.BadLog;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.enums.CamMode;
import frc.robot.enums.DriveState;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class Aim extends CommandBase {

    private final DriveSubsystem mDriveSubsystem;
    private final VisionSubsystem mVisionSubsystem;
    private final PIDController mPidController;
    private double[] mTargetAngles;
    private double[] mGyroAngles;
    private boolean[] mValidAngles;
    private double mTargetAngle;
    private CamMode mCamMode;
    private int mIteration;

    public Aim(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, int samples) {
        mDriveSubsystem = driveSubsystem;
        mVisionSubsystem = visionSubsystem;
        mPidController = new PIDController(Constants.Vision.kPAim, Constants.Vision.kIAim, Constants.Vision.kDAim);

        mTargetAngles = new double[samples];
        mGyroAngles = new double[samples];
        mValidAngles = new boolean[samples];

        mTargetAngle = 0.0;

        // Require DriveSubsystem and VisionSubsystem
        addRequirements(mDriveSubsystem, mVisionSubsystem);
    }

    @Override
    public void initialize() {
        mIteration = 0;
        mTargetAngle = 0.0;
        mCamMode = mVisionSubsystem.getCamMode();
        mVisionSubsystem.setCamMode(CamMode.CV);
        mVisionSubsystem.setLED(true);
        mDriveSubsystem.brake();
        mDriveSubsystem.setState(DriveState.AIM);
    }


    @Override
    public void execute() {
        if(mIteration == mTargetAngles.length) {
            int recognizedFrames = 0;
            for (int i = 0; i < mTargetAngles.length; i++) {
                if(mValidAngles[i]) {
                    mTargetAngle += mGyroAngles[i] - mTargetAngles[i];
                    recognizedFrames++;
                }
            }
            if(recognizedFrames == 0) {
                mTargetAngle = mDriveSubsystem.getUnboundedHeading();
            } else {
                mTargetAngle /= recognizedFrames;
            }
            mPidController.setSetpoint(Constants.Vision.kAimOffset +  mTargetAngle);
        }

        if(mIteration < mTargetAngles.length) {
            if(mVisionSubsystem.targetFound()) {
                mTargetAngles[mIteration] = mVisionSubsystem.getTargetCenter().getX();
                mGyroAngles[mIteration] = mDriveSubsystem.getUnboundedHeading();
                mValidAngles[mIteration] = true;
            } else {
                mTargetAngles[mIteration] = 0;
                mGyroAngles[mIteration] = 0;
                mValidAngles[mIteration] = false;
            }
        } else {
            double power = mPidController.calculate(mDriveSubsystem.getUnboundedHeading());
            power = Math.abs(Math.min(Constants.Vision.kMaxAimPower, Math.abs(power))) * Math.signum(power);
            mDriveSubsystem.setPowers(-power, power);
        }
        mIteration++;
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