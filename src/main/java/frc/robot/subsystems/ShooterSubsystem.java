package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.MotorConfig;
import frc.lib.motion.EncoderUtil;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

    private WPI_TalonFX m_leftMotor;
    private WPI_TalonFX m_rightMotor;

    public ShooterSubsystem() {
        m_leftMotor = new WPI_TalonFX(Constants.kLeftShooterMotor);
        m_rightMotor = new WPI_TalonFX(Constants.kRightShooterMotor);

        MotorConfig.configTalon(m_leftMotor, Constants.kShooterConfig, Constants.kShooterSlot);
        MotorConfig.configTalon(m_rightMotor, Constants.kShooterConfig, Constants.kShooterSlot);

        m_leftMotor.setNeutralMode(NeutralMode.Coast);
        m_rightMotor.setNeutralMode(NeutralMode.Coast);
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

    public void spin(double rpm) {
        double velocity = EncoderUtil.fromRPM(rpm, Constants.kShooterEncoderUnitsPerRev, Constants.kShooterGearRatio, Constants.kShooterVelocitySampleTime);
        m_leftMotor.set(ControlMode.Velocity, velocity);
        m_rightMotor.set(ControlMode.Velocity, velocity);
    }

    public void coast() {
        m_leftMotor.set(ControlMode.PercentOutput, 0.0);
        m_rightMotor.set(ControlMode.PercentOutput, 0.0);
    }

    public double getRPM() {
        return EncoderUtil.toRPM(m_rightMotor.getSelectedSensorVelocity(), Constants.kShooterEncoderUnitsPerRev, Constants.kShooterGearRatio, Constants.kShooterVelocitySampleTime);
    }
}