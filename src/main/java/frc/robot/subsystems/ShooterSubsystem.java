package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.motion.EncoderUtil;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

    private WPI_TalonFX m_leftMotor;
    private WPI_TalonFX m_rightMotor;

    public ShooterSubsystem(int leftTalonID, int rightTalonID) {
        m_leftMotor = new WPI_TalonFX(leftTalonID);
        m_rightMotor = new WPI_TalonFX(rightTalonID);

        m_leftMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    }

    @Override
    public void periodic() {
        
    }

    public void leftInverted(boolean inverted) {
        m_leftMotor.setInverted(inverted);
    }

    public void rightInverted(boolean inverted) {
        m_rightMotor.setInverted(inverted);
    }

    public void configPID(double p, double i, double d) {
        m_rightMotor.config_kP(0, p);
        m_rightMotor.config_kI(0, i);
        m_rightMotor.config_kD(0, d);
        m_leftMotor.config_kP(0, p);
        m_leftMotor.config_kI(0, i);
        m_leftMotor.config_kD(0, d);
    }

    public void spin(double rpm) {
        double velocity = EncoderUtil.fromRPM(rpm, Constants.kShooterEncoderUnitsPerRev, Constants.kShooterGearRatio, Constants.kShooterVelocitySampleTime);
        m_leftMotor.set(ControlMode.Velocity, velocity);
        m_rightMotor.set(ControlMode.Velocity, velocity);
    }
}