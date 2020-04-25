/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.sensors.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.kinematics.*;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.config.FeedbackConfig;
import frc.lib.config.FeedbackConstants;
import frc.lib.config.MotorConfig;
import frc.lib.motion.*;
import frc.lib.util.Util;
import frc.robot.*;


public class DriveSubsystem extends SubsystemBase {

    private final WPI_TalonFX mRight = new WPI_TalonFX(Constants.Drive.kRightMotorPort);
    private final WPI_TalonFX mRightFollower = new WPI_TalonFX(Constants.Drive.kRightMotorFollowerPort);
    private final WPI_TalonFX mLeft = new WPI_TalonFX(Constants.Drive.kLeftMotorPort);
    private final WPI_TalonFX mLeftFollower = new WPI_TalonFX(Constants.Drive.kLeftMotorFollowerPort);

    private final SpeedControllerGroup mRightMotors = new SpeedControllerGroup(mRight, mRightFollower);
    private final SpeedControllerGroup mLeftMotors = new SpeedControllerGroup(mLeft, mLeftFollower);

    private final DifferentialDrive mDrive = new DifferentialDrive(mLeftMotors, mRightMotors);
    private final DifferentialDriveOdometry mOdometry;

    private final PigeonIMU mImu = new PigeonIMU(0);

    private ShuffleboardTab mRobotTab = Shuffleboard.getTab("Robot");
    
    public DriveSubsystem() {
        mRight.setInverted(Constants.Drive.kRightInverted);
        mLeft.setInverted(Constants.Drive.kLeftInverted);
        mRightFollower.setInverted(Constants.Drive.kRightInverted);
        mLeftFollower.setInverted(Constants.Drive.kLeftInverted);

        mDrive.setRightSideInverted(Constants.Drive.kRightInverted);

        mDrive.setSafetyEnabled(false);

        mRobotTab.add("Drive", mDrive).withWidget(BuiltInWidgets.kDifferentialDrive);

        mImu.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_1_General, Constants.Gyro.kPigeonCondFrame1Period);
        mImu.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_6_SensorFusion, Constants.Gyro.kPigeonCondFrame6Period);
        mImu.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, Constants.Gyro.kPigeonCondFrame9Period);
        mImu.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_11_GyroAccum, Constants.Gyro.kPigeonCondFrame11Period);
        mImu.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_3_GeneralAccel, Constants.Gyro.kPigeonCondFrame3Period);
        mImu.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_10_SixDeg_Quat, Constants.Gyro.kPigeonCondFrame10Period);
        mImu.setStatusFramePeriod(PigeonIMU_StatusFrame.RawStatus_4_Mag, Constants.Gyro.kPigeonRawFrame4Period);
        mImu.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_2_Gyro, Constants.Gyro.kPigeonBiasedFrame2Period);
        mImu.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_6_Accel, Constants.Gyro.kPigeonBiasedFrame6Period);

        brake();

        configTalons();
        resetHeading();
        resetEncoders();
        mOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    }


    @Override
    public void periodic() {
        mOdometry.update(Rotation2d.fromDegrees(getHeading()), getPositionLeft(), getPositionRight());
    }

    public void setPowers(double powerL, double powerR) {
        mRight.set(powerR);
        mLeft.set(powerL);
    }

    public void setVoltage(double voltsL, double voltsR) {
        mRight.setVoltage(voltsR);
        mLeft.setVoltage(voltsL);
    }

    public void setVoltageReverse(double voltsR, double voltsL) {
        mRightMotors.setVoltage(-voltsR);
        mLeftMotors.setVoltage(-voltsL);
    }

    public void setArcade(double throttle, double turn) {
        mRightMotors.set(throttle - turn);
        mLeftMotors.set(throttle + turn);
    }

    public void driveClosedLoop(double left, double right) {
        mRight.set(ControlMode.Velocity, fromVelocity(right));
        mLeft.set(ControlMode.Velocity, fromVelocity(left));
    }

    public void brake() {
        mLeft.setNeutralMode(NeutralMode.Brake);
        mRight.setNeutralMode(NeutralMode.Brake);
    }

    public void coast() {
        mLeft.setNeutralMode(NeutralMode.Coast);
        mRight.setNeutralMode(NeutralMode.Coast);
    }

    public void hold() {
        mLeft.set(ControlMode.Position, 0.0);
        mRight.set(ControlMode.Position, 0.0);
    }

    public Pose2d getPose() {
        return mOdometry.getPoseMeters();
    }

    public Pose2d getPoseReversed() {
        return new Pose2d(mOdometry.getPoseMeters().getTranslation(), mOdometry.getPoseMeters().getRotation().rotateBy(new Rotation2d(Math.toRadians(-180))));
    }

    public double getHeading() {
        return Util.boundedAngleDegrees(mImu.getFusedHeading(), false) * (Constants.Gyro.kGyroReversed ? -1.0 : 1.0);
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getVelocityLeft(), getVelocityRight());
    }

    public DifferentialDriveWheelSpeeds getWheelSpeedsReverse() {
        return new DifferentialDriveWheelSpeeds(-getVelocityRight(), -getVelocityLeft());
    }

    public void reset() {
        resetOdometry(new Pose2d());
        configTalons();
        resetHeading();
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        mOdometry.resetPosition(pose, pose.getRotation());
    }

    public void resetGyro() {
        mImu.setFusedHeading(0);
    }

    public void resetEncoders() {
        resetEncoder(mRight);
        resetEncoder(mLeft);
    }

    private void resetEncoder(TalonFX talon) {
        talon.setSelectedSensorPosition(0);
    }

    private void resetHeading() {
        mImu.setFusedHeading(0);
    }

    public void resetTalons() {
        mRight.configFactoryDefault();
        mRightFollower.configFactoryDefault();
        mLeft.configFactoryDefault();
        mLeftFollower.configFactoryDefault();
        mRight.set(ControlMode.PercentOutput, 0.0);
        mLeft.set(ControlMode.PercentOutput, 0.0);
    }

    public void configTalons() {
        resetTalons();
        mRightFollower.follow(mRight);
        mLeftFollower.follow(mLeft);
        mRight.configMotionProfileTrajectoryPeriod(0);
        mLeft.configMotionProfileTrajectoryPeriod(0);
        MotorConfig.resetTalon(mRight);
        MotorConfig.resetTalon(mLeft);
        FeedbackConstants.config(mRight, Constants.Drive.kFeedbackConstants, Constants.Drive.kSlot);
        FeedbackConstants.config(mLeft, Constants.Drive.kFeedbackConstants, Constants.Drive.kSlot);
        FeedbackConfig.config(mRight, Constants.Drive.kFeedbackConfig);
        FeedbackConfig.config(mLeft, Constants.Drive.kFeedbackConfig);
        MotorConfig.config(mRight, Constants.Drive.kConfig);
        MotorConfig.config(mLeft, Constants.Drive.kConfig);
    }

    public double getPositionLeft() {
        return toDistance(mLeft.getSelectedSensorPosition());
    }

    public double getPositionRight() {
        return toDistance(mRight.getSelectedSensorPosition());
    }

    public double getAverageDistance() {
        return (getPositionLeft() + getPositionRight()) / 2.0;
    }

    public double toDistance(int sensorPosition) {
        return EncoderUtil.toDistance(sensorPosition, Constants.Drive.kFeedbackConfig.getEpr(), Constants.Drive.kFeedbackConfig.getGearRatio(), Constants.Drive.kWheelDiameter);
    }

    public double getVelocityRight() {
        return toVelocity(mRight.getSelectedSensorVelocity());
    }

    public double getVelocityLeft() {
        return toVelocity(mLeft.getSelectedSensorVelocity());
    }

    public double toVelocity(int velocity) {
        return EncoderUtil.toVelocity(velocity, Constants.Drive.kFeedbackConfig.getEpr(), Constants.Drive.kFeedbackConfig.getGearRatio(), Constants.Drive.kWheelDiameter);
    }

    public double fromVelocity(double velocity) {
        return EncoderUtil.fromVelocity(velocity, Constants.Drive.kFeedbackConfig.getEpr(), Constants.Drive.kFeedbackConfig.getGearRatio(), Constants.Drive.kWheelDiameter);
    }

    public void setMaxOutput(double maxOutput) {
        mDrive.setMaxOutput(maxOutput);
    }

    public WPI_TalonFX[] getAllTalons() {
        return new WPI_TalonFX[]{ mRight, mRightFollower, mLeft, mLeftFollower };
    }

    public DifferentialDriveOdometry getOdometry() {
        return mOdometry;
    }

    public PigeonIMU getGyro() {
        return mImu;
    }

    public double[] getTemps() {
        double[] temperatures = { mRight.getTemperature(), mRightFollower.getTemperature(), mLeft.getTemperature(), mLeftFollower.getTemperature() };
        return temperatures;
    }

}