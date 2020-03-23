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
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.kinematics.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.config.MotorConfig;
import frc.robot.*;


public class SwerveDriveSubsystem extends SubsystemBase {

    private final WPI_TalonFX mFL = new WPI_TalonFX(0);
    private final WPI_TalonFX mFR = new WPI_TalonFX(0);
    private final WPI_TalonFX mBL = new WPI_TalonFX(0);
    private final WPI_TalonFX mBR = new WPI_TalonFX(0);

    private final WPI_TalonSRX mFLTurn = new WPI_TalonSRX(0);
    private final WPI_TalonSRX mFRTurn = new WPI_TalonSRX(0);
    private final WPI_TalonSRX mBLTurn = new WPI_TalonSRX(0);
    private final WPI_TalonSRX mBRTurn = new WPI_TalonSRX(0);

    private final SwerveDriveKinematics mKinematics;
    private final SwerveDriveOdometry mOdometry;

    private final PigeonIMU mImu = new PigeonIMU(0);
    
    public SwerveDriveSubsystem() {
        mFL.setInverted(false);
        mFR.setInverted(false);
        mBL.setInverted(false);
        mBR.setInverted(false);

        mFLTurn.setInverted(false);
        mFRTurn.setInverted(false);
        mBLTurn.setInverted(false);
        mBRTurn.setInverted(false);

        configTalons();
        resetHeading();
        resetEncoders();

        double width = 0;
        double height = 0;

        mKinematics = new SwerveDriveKinematics(
            new Translation2d(-width/2.0, height/2.0),
            new Translation2d(width/2.0, height/2.0),
            new Translation2d(-width/2.0, -height/2.0),
            new Translation2d(width/2.0, -height/2.0));

        mOdometry = new SwerveDriveOdometry(mKinematics, new Rotation2d(0), new Pose2d());
    }


    @Override
    public void periodic() {
        mOdometry.update(Rotation2d.fromDegrees(getHeading()),
            new SwerveModuleState(0.0, new Rotation2d()),
            new SwerveModuleState(0.0, new Rotation2d()),
            new SwerveModuleState(0.0, new Rotation2d()),
            new SwerveModuleState(0.0, new Rotation2d()));
    }

    /**
     * Set the states of the modules
     * @param moduleStates The states of the modules(FL, FR, BL, BR)
     */
    public void setStates(SwerveModuleState... moduleStates) {
        
    }

    /**
     * Drive with the specified parameters
     * @param tX Translational x movement
     * @param tY Translational y movement
     * @param twist Twist(rotational) motion
     */
    public void drive(double tX, double tY, double twist) {

    }

    public void brake() {
        mFL.setNeutralMode(NeutralMode.Brake);
        mFR.setNeutralMode(NeutralMode.Brake);
        mBL.setNeutralMode(NeutralMode.Brake);
        mBR.setNeutralMode(NeutralMode.Brake);
        mFLTurn.setNeutralMode(NeutralMode.Brake);
        mFRTurn.setNeutralMode(NeutralMode.Brake);
        mBLTurn.setNeutralMode(NeutralMode.Brake);
        mBRTurn.setNeutralMode(NeutralMode.Brake);
    }

    public void coast() {
        mFL.setNeutralMode(NeutralMode.Coast);
        mFR.setNeutralMode(NeutralMode.Coast);
        mBL.setNeutralMode(NeutralMode.Coast);
        mBR.setNeutralMode(NeutralMode.Coast);
    }

    public void hold() {
        mFL.set(ControlMode.Position, mFL.getSelectedSensorPosition());
        mFR.set(ControlMode.Position, mFR.getSelectedSensorPosition());
        mBL.set(ControlMode.Position, mBL.getSelectedSensorPosition());
        mBR.set(ControlMode.Position, mBR.getSelectedSensorPosition());
    }

    public Pose2d getPose() {
        return mOdometry.getPoseMeters();
    }

    public double getHeading() {
        return Math.IEEEremainder(mImu.getFusedHeading(), 360) * (Constants.Gyro.kGyroReversed ? -1.0 : 1.0);
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
        resetEncoder(mFL);
        resetEncoder(mFR);
        resetEncoder(mBL);
        resetEncoder(mBR);
        resetEncoder(mFLTurn);
        resetEncoder(mFRTurn);
        resetEncoder(mBLTurn);
        resetEncoder(mBRTurn);
    }

    private void resetEncoder(BaseMotorController talon) {
        talon.setSelectedSensorPosition(0);
    }

    private void resetHeading() {
        mImu.setFusedHeading(0);
    }

    public void resetTalons() {
        mFL.configFactoryDefault();
        mFR.configFactoryDefault();
        mBL.configFactoryDefault();
        mBR.configFactoryDefault();
        mFLTurn.configFactoryDefault();
        mFRTurn.configFactoryDefault();
        mBLTurn.configFactoryDefault();
        mBRTurn.configFactoryDefault();
    }

    public void configTalons() {
        resetTalons();
        mFL.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        mFR.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        mBL.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        mBR.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        mFLTurn.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        mFRTurn.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        mBLTurn.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        mBRTurn.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        MotorConfig.resetTalon(mFL);
        MotorConfig.resetTalon(mFR);
        MotorConfig.resetTalon(mBL);
        MotorConfig.resetTalon(mBR);
        MotorConfig.resetTalon(mFLTurn);
        MotorConfig.resetTalon(mFRTurn);
        MotorConfig.resetTalon(mBLTurn);
        MotorConfig.resetTalon(mBRTurn);
        //FeedbackConstants.config(mFL, Constants.Drive., Constants.Drive.kSlot);
        //FeedbackConstants.config(mFR, Constants.Drive., Constants.Drive.kSlot);
        //FeedbackConstants.config(mBL, Constants.Drive., Constants.Drive.kSlot);
        //FeedbackConstants.config(mBR, Constants.Drive., Constants.Drive.kSlot);
        //FeedbackConstants.config(mFLTurn, Constants.Drive., Constants.Drive.kSlot);
        //FeedbackConstants.config(mFRTurn, Constants.Drive., Constants.Drive.kSlot);
        //FeedbackConstants.config(mBLTurn, Constants.Drive., Constants.Drive.kSlot);
        //FeedbackConstants.config(mBRTurn, Constants.Drive., Constants.Drive.kSlot);
        //FeedbackConfig.config(mFL, Constants.Drive.kGearRatio, Constants.Drive.kEPR);
        //FeedbackConfig.config(mFR, Constants.Drive.kGearRatio, Constants.Drive.kEPR);
        //FeedbackConfig.config(mBL, Constants.Drive.kGearRatio, Constants.Drive.kEPR);
        //FeedbackConfig.config(mBR, Constants.Drive.kGearRatio, Constants.Drive.kEPR);
        //FeedbackConfig.config(mFLTurn, Constants.Drive.kTurnGearRatio, Constants.Drive.kTurnEPR);
        //FeedbackConfig.config(mFRTurn, Constants.Drive.kTurnGearRatio, Constants.Drive.kTurnEPR);
        //FeedbackConfig.config(mBLTurn, Constants.Drive.kTurnGearRatio, Constants.Drive.kTurnEPR);
        //FeedbackConfig.config(mBRTurn, Constants.Drive.kTurnGearRatio, Constants.Drive.kTurnEPR);
        //MotorConfig.config(mFL, Constants.Drive.kConfig);
        //MotorConfig.config(mFR, Constants.Drive.kConfig);
        //MotorConfig.config(mBL, Constants.Drive.kConfig);
        //MotorConfig.config(mBR, Constants.Drive.kConfig);
        //MotorConfig.config(mFLTurn, Constants.Drive.kTurnConfig);
        //MotorConfig.config(mFRTurn, Constants.Drive.kTurnConfig);
        //MotorConfig.config(mBLTurn, Constants.Drive.kTurnConfig);
        //MotorConfig.config(mBRTurn, Constants.Drive.kTurnConfig);
    }

}