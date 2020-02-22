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
import frc.lib.config.MotorConfig;
import frc.lib.motion.*;
import frc.robot.*;


public class DriveSubsystem extends SubsystemBase {

    private final WPI_TalonFX m_right = new WPI_TalonFX(Constants.kRightDriveMotorPort);
    private final WPI_TalonFX m_rightFollower = new WPI_TalonFX(Constants.kRightDriveMotorFollowerPort);
    private final WPI_TalonFX m_left = new WPI_TalonFX(Constants.kLeftDriveMotorPort);
    private final WPI_TalonFX m_leftFollower = new WPI_TalonFX(Constants.kLeftDriveMotorFollowerPort);

    private final SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(m_right, m_rightFollower);
    private final SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(m_left, m_leftFollower);

    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
    private final DifferentialDriveOdometry m_odometry;

    private final PigeonIMU m_imu = new PigeonIMU(0);

    private final Solenoid m_cooling = new Solenoid(Constants.kCoolingSolenoid);
    private final Timer m_coolingTimer = new Timer();

    private ShuffleboardTab m_robotTab = Shuffleboard.getTab("Robot");
    
    public DriveSubsystem() {
        m_right.setSensorPhase(Constants.kRightSensorInverted);
        m_left.setSensorPhase(Constants.kLeftSensorInverted);

        m_drive.setRightSideInverted(Constants.kRightInverted);

        m_drive.setSafetyEnabled(false);

        m_robotTab.add("Drive", m_drive).withWidget(BuiltInWidgets.kDifferentialDrive);

        coast();

        configTalons();
        resetHeading();
        resetEncoders();
        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
        m_coolingTimer.start();
    }


    @Override
    public void periodic() {
        m_odometry.update(Rotation2d.fromDegrees(getHeading()), getPositionLeft(), getPositionRight());
        double[] temps = getTemps();
        if(m_coolingTimer.hasPeriodPassed(Constants.kCoolingDelay)) {
            m_cooling.set(false);
        }
        for (int i = 0; i < temps.length; i++) {
            if(temps[i] >= Constants.kCoolingTemp) {
                m_cooling.set(true);
                m_coolingTimer.reset();
                break;
            }
        }
    }

    public void setPowers(double powerL, double powerR) {
        m_rightMotors.set(powerR);
        m_leftMotors.set(powerL);
    }

    public void setVoltage(double voltsL, double voltsR) {
        m_rightMotors.setVoltage(voltsR);
        m_leftMotors.setVoltage(voltsL);
    }

    public void setArcade(double throttle, double turn) {
        m_rightMotors.set(throttle - turn);
        m_leftMotors.set(throttle + turn);
    }

    public void driveClosedLoop(int left, int right) {
        m_right.set(ControlMode.Velocity, toVelocity(right));
        m_left.set(ControlMode.Velocity, toVelocity(left));
    }

    public void brake() {
        m_left.setNeutralMode(NeutralMode.Brake);
        m_right.setNeutralMode(NeutralMode.Brake);
    }

    public void coast() {
        m_left.setNeutralMode(NeutralMode.Coast);
        m_right.setNeutralMode(NeutralMode.Coast);
    }

    public void hold() {
        m_left.set(ControlMode.Position, 0.0);
        m_right.set(ControlMode.Position, 0.0);
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public double getHeading() {
        return Math.IEEEremainder(m_imu.getFusedHeading(), 360) * (Constants.kGyroReversed ? -1.0 : 1.0);
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getVelocityLeft(), getVelocityRight());
    }

    public void reset() {
        resetOdometry(new Pose2d());
        resetTalons();
        resetHeading();
        resetGyro();

        
        m_right.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_1Ms);
        m_left.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_1Ms);
        m_right.configVelocityMeasurementWindow(1);
        m_left.configVelocityMeasurementWindow(1);
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }

    public void resetGyro() {
        m_imu.setFusedHeading(0);
    }

    public void resetEncoders() {
        resetEncoder(m_right);
        resetEncoder(m_left);
    }

    private void resetEncoder(TalonFX talon) {
        talon.setSelectedSensorPosition(0);
    }

    private void resetHeading() {
        m_imu.setFusedHeading(0);
    }

    public void resetTalons() {
        m_right.configFactoryDefault();
        m_rightFollower.configFactoryDefault();
        m_left.configFactoryDefault();
        m_leftFollower.configFactoryDefault();
        m_right.set(ControlMode.PercentOutput, 0.0);
        m_left.set(ControlMode.PercentOutput, 0.0);
    }

    public void configTalons() {
        resetTalons();
        m_rightFollower.follow(m_right);
        m_leftFollower.follow(m_left);
        m_right.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        m_left.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        m_right.configMotionProfileTrajectoryPeriod(0);
        m_left.configMotionProfileTrajectoryPeriod(0);
        MotorConfig.configTalon(m_right, Constants.kDriveConfig, Constants.kDriveSlot);
        MotorConfig.configTalon(m_left, Constants.kDriveConfig, Constants.kDriveSlot);
    }

    public double getPositionLeft() {
        return toDistance(m_left.getSelectedSensorPosition());
    }

    public double getPositionRight() {
        return toDistance(m_right.getSelectedSensorPosition());
    }

    public double getAverageDistance() {
        return (getPositionLeft() + getPositionRight()) / 2.0;
    }

    public double toDistance(int sensorPosition) {
        return EncoderUtil.toDistance(sensorPosition, Constants.kDriveEncoderUnitsPerRev, Constants.kDriveGearRatio, Constants.kDriveWheelDiameter);
    }

    public double getVelocityRight() {
        return toVelocity(m_right.getSelectedSensorVelocity());
    }

    public double getVelocityLeft() {
        return toVelocity(m_left.getSelectedSensorVelocity());
    }

    public double toVelocity(int velocity) {
        return EncoderUtil.toVelocity(velocity, Constants.kDriveEncoderUnitsPerRev, Constants.kDriveGearRatio, Constants.kDriveWheelDiameter, Constants.kVelSampleTime);
    }

    public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
    }

    public WPI_TalonFX[] getAllTalons() {
        return new WPI_TalonFX[]{ m_right, m_rightFollower, m_left, m_leftFollower };
    }

    public DifferentialDriveOdometry getOdometry() {
        return m_odometry;
    }

    public PigeonIMU getGyro() {
        return m_imu;
    }

    public double[] getTemps() {
        double[] temperatures = { m_right.getTemperature(), m_rightFollower.getTemperature(), m_left.getTemperature(), m_leftFollower.getTemperature() };
        return temperatures;
    }

}