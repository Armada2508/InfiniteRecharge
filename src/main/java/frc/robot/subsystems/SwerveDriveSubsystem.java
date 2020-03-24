/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source SoFRware - may be modified and shared by FLC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.sensors.*;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.kinematics.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.config.MotorConfig;
import frc.lib.drive.SwerveDrive;
import frc.robot.*;


public class SwerveDriveSubsystem extends SubsystemBase {

    private final WPI_TalonFX mFR = new WPI_TalonFX(0);
    private final WPI_TalonFX mFL = new WPI_TalonFX(0);
    private final WPI_TalonFX mBL = new WPI_TalonFX(0);
    private final WPI_TalonFX mBR = new WPI_TalonFX(0);

    private final WPI_TalonSRX mFRTurn = new WPI_TalonSRX(0);
    private final WPI_TalonSRX mFLTurn = new WPI_TalonSRX(0);
    private final WPI_TalonSRX mBLTurn = new WPI_TalonSRX(0);
    private final WPI_TalonSRX mBRTurn = new WPI_TalonSRX(0);

    private final SwerveDriveKinematics mKinematics;
    private final SwerveDriveOdometry mOdometry;
    private final SwerveDrive mDrive;
    private SwerveModuleState[] mModuleStates;

    private final PigeonIMU mImu = new PigeonIMU(0);
    
    public SwerveDriveSubsystem() {
        mFR.setInverted(false);
        mFL.setInverted(false);
        mBL.setInverted(false);
        mBR.setInverted(false);

        mFRTurn.setInverted(false);
        mFLTurn.setInverted(false);
        mBLTurn.setInverted(false);
        mBRTurn.setInverted(false);

        configTalons();
        resetHeading();
        resetEncoders();

        double width = 0;
        double height = 0;
        
        Translation2d[] wheelPositions = {
            new Translation2d(width/2.0, height/2.0),
            new Translation2d(-width/2.0, height/2.0),
            new Translation2d(-width/2.0, -height/2.0),
            new Translation2d(width/2.0, -height/2.0)};

        mKinematics = new SwerveDriveKinematics(wheelPositions);

        mDrive = new SwerveDrive(wheelPositions);

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
     * @param moduleStates The states of the modules(FR, FL, BL, BR)
     */
    public void setModuleStates(SwerveModuleState... moduleStates) {
        mModuleStates = moduleStates.clone();
    }

    /**
     * @return The states of the swerve modules
     */
    public SwerveModuleState[] getModuleStates() {
        return mModuleStates;
    }

    /**
     * Drive with the specified parameters
     * @param tX Translational x movement
     * @param tY Translational y movement
     * @param twist Twist(rotational) motion
     */
    public void drive(double tX, double tY, double twist) {
        setModuleStates(mDrive.calculate(tX, tY, twist, 0.0));
    }

    public void brake() {
        mFR.setNeutralMode(NeutralMode.Brake);
        mFL.setNeutralMode(NeutralMode.Brake);
        mBL.setNeutralMode(NeutralMode.Brake);
        mBR.setNeutralMode(NeutralMode.Brake);
        mFRTurn.setNeutralMode(NeutralMode.Brake);
        mFLTurn.setNeutralMode(NeutralMode.Brake);
        mBLTurn.setNeutralMode(NeutralMode.Brake);
        mBRTurn.setNeutralMode(NeutralMode.Brake);
    }

    public void coast() {
        mFR.setNeutralMode(NeutralMode.Coast);
        mFL.setNeutralMode(NeutralMode.Coast);
        mBL.setNeutralMode(NeutralMode.Coast);
        mBR.setNeutralMode(NeutralMode.Coast);
    }

    public void hold() {
        mFR.set(ControlMode.Position, mFR.getSelectedSensorPosition());
        mFL.set(ControlMode.Position, mFL.getSelectedSensorPosition());
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
        resetEncoder(mFR);
        resetEncoder(mFL);
        resetEncoder(mBL);
        resetEncoder(mBR);
        resetEncoder(mFRTurn);
        resetEncoder(mFLTurn);
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
        mFR.configFactoryDefault();
        mFL.configFactoryDefault();
        mBL.configFactoryDefault();
        mBR.configFactoryDefault();
        mFRTurn.configFactoryDefault();
        mFLTurn.configFactoryDefault();
        mBLTurn.configFactoryDefault();
        mBRTurn.configFactoryDefault();
    }

    public void configTalons() {
        resetTalons();
        mFR.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        mFL.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        mBL.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        mBR.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        mFRTurn.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        mFLTurn.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        mBLTurn.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        mBRTurn.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        MotorConfig.resetTalon(mFR);
        MotorConfig.resetTalon(mFL);
        MotorConfig.resetTalon(mBL);
        MotorConfig.resetTalon(mBR);
        MotorConfig.resetTalon(mFRTurn);
        MotorConfig.resetTalon(mFLTurn);
        MotorConfig.resetTalon(mBLTurn);
        MotorConfig.resetTalon(mBRTurn);
        //FeedbackConstants.config(mFR, Constants.Drive., Constants.Drive.kSlot);
        //FeedbackConstants.config(mFL, Constants.Drive., Constants.Drive.kSlot);
        //FeedbackConstants.config(mBL, Constants.Drive., Constants.Drive.kSlot);
        //FeedbackConstants.config(mBR, Constants.Drive., Constants.Drive.kSlot);
        //FeedbackConstants.config(mFRTurn, Constants.Drive., Constants.Drive.kSlot);
        //FeedbackConstants.config(mFLTurn, Constants.Drive., Constants.Drive.kSlot);
        //FeedbackConstants.config(mBLTurn, Constants.Drive., Constants.Drive.kSlot);
        //FeedbackConstants.config(mBRTurn, Constants.Drive., Constants.Drive.kSlot);
        //FeedbackConfig.config(mFR, Constants.Drive.kGearRatio, Constants.Drive.kEPR);
        //FeedbackConfig.config(mFL, Constants.Drive.kGearRatio, Constants.Drive.kEPR);
        //FeedbackConfig.config(mBL, Constants.Drive.kGearRatio, Constants.Drive.kEPR);
        //FeedbackConfig.config(mBR, Constants.Drive.kGearRatio, Constants.Drive.kEPR);
        //FeedbackConfig.config(mFRTurn, Constants.Drive.kTurnGearRatio, Constants.Drive.kTurnEPR);
        //FeedbackConfig.config(mFLTurn, Constants.Drive.kTurnGearRatio, Constants.Drive.kTurnEPR);
        //FeedbackConfig.config(mBLTurn, Constants.Drive.kTurnGearRatio, Constants.Drive.kTurnEPR);
        //FeedbackConfig.config(mBRTurn, Constants.Drive.kTurnGearRatio, Constants.Drive.kTurnEPR);
        //MotorConfig.config(mFR, Constants.Drive.kConfig);
        //MotorConfig.config(mFL, Constants.Drive.kConfig);
        //MotorConfig.config(mBL, Constants.Drive.kConfig);
        //MotorConfig.config(mBR, Constants.Drive.kConfig);
        //MotorConfig.config(mFRTurn, Constants.Drive.kTurnConfig);
        //MotorConfig.config(mFLTurn, Constants.Drive.kTurnConfig);
        //MotorConfig.config(mBLTurn, Constants.Drive.kTurnConfig);
        //MotorConfig.config(mBRTurn, Constants.Drive.kTurnConfig);
    }

}