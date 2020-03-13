package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.MotorConfig;
import frc.lib.motion.EncoderUtil;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

    private WPI_TalonFX mLeftMotor;
    private WPI_TalonFX mRightMotor;

    public ShooterSubsystem() {
        mLeftMotor = new WPI_TalonFX(Constants.Shooter.kLeftShooterMotor);
        mRightMotor = new WPI_TalonFX(Constants.Shooter.kRightShooterMotor);

        MotorConfig.resetTalon(mLeftMotor);
        MotorConfig.resetTalon(mRightMotor);

        MotorConfig.configTalon(mLeftMotor, Constants.Shooter.kShooterConfig, Constants.Shooter.kShooterSlot);
        MotorConfig.configTalon(mRightMotor, Constants.Shooter.kShooterConfig, Constants.Shooter.kShooterSlot);

        mLeftMotor.setNeutralMode(NeutralMode.Coast);
        mRightMotor.setNeutralMode(NeutralMode.Coast);
    }

    @Override
    public void periodic() {
    }

    public void leftInverted(boolean inverted) {
        mLeftMotor.setInverted(inverted);
    }

    public void rightInverted(boolean inverted) {
        mRightMotor.setInverted(inverted);
    }

    public void spin(double rpm) {
        double velocity = EncoderUtil.fromRPM(rpm, Constants.Shooter.kShooterEncoderUnitsPerRev, Constants.Shooter.kShooterGearRatio, Constants.Shooter.kShooterVelocitySampleTime);
        mLeftMotor.set(ControlMode.Velocity, velocity);
        mRightMotor.set(ControlMode.Velocity, velocity);
    }

    public void coast() {
        mLeftMotor.set(ControlMode.PercentOutput, 0.0);
        mRightMotor.set(ControlMode.PercentOutput, 0.0);
    }

    public double getRPM() {
        return EncoderUtil.toRPM(mRightMotor.getSelectedSensorVelocity(), Constants.Shooter.kShooterEncoderUnitsPerRev, Constants.Shooter.kShooterGearRatio, Constants.Shooter.kShooterVelocitySampleTime);
    }

    public void setCurrent(double amps) {
        //mLeftMotor.configGetStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, amps, 0, 0));
    }
}