package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.FeedbackConstants;
import frc.lib.config.MotorConfig;
import frc.lib.motion.EncoderUtil;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

    private WPI_TalonFX mLeftMotor;
    private WPI_TalonFX mRightMotor;

    public ShooterSubsystem() {
        mLeftMotor = new WPI_TalonFX(Constants.Shooter.kLeftMotor);
        mRightMotor = new WPI_TalonFX(Constants.Shooter.kRightMotor);

        MotorConfig.resetTalon(mLeftMotor);
        MotorConfig.resetTalon(mRightMotor);
        
        leftInverted(Constants.Shooter.kLeftInverted);
        rightInverted(Constants.Shooter.kRightInverted);

        FeedbackConstants.config(mLeftMotor, Constants.Shooter.kFeedbackConstants, Constants.Shooter.kSlot);
        FeedbackConstants.config(mRightMotor, Constants.Shooter.kFeedbackConstants, Constants.Shooter.kSlot);

        MotorConfig.config(mLeftMotor, Constants.Shooter.kConfig);
        MotorConfig.config(mRightMotor, Constants.Shooter.kConfig);

        mLeftMotor.setNeutralMode(NeutralMode.Coast);
        mRightMotor.setNeutralMode(NeutralMode.Coast);

        mLeftMotor.configClosedloopRamp(Constants.Shooter.kRamp);
        mLeftMotor.configOpenloopRamp(Constants.Shooter.kRamp);
        mRightMotor.configClosedloopRamp(Constants.Shooter.kRamp);
        mRightMotor.configOpenloopRamp(Constants.Shooter.kRamp);
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
        double velocity = EncoderUtil.fromRPM(rpm, Constants.Shooter.kFeedbackConfig.getEpr(), Constants.Shooter.kFeedbackConfig.getGearRatio());
        mLeftMotor.set(ControlMode.Velocity, velocity);
        mRightMotor.set(ControlMode.Velocity, velocity);
    }

    public void setPower(double power) {
        mLeftMotor.set(ControlMode.PercentOutput, power);
        mRightMotor.set(ControlMode.PercentOutput, power);
    }

    public void coast() {
        mLeftMotor.set(ControlMode.PercentOutput, 0.0);
        mRightMotor.set(ControlMode.PercentOutput, 0.0);
    }

    public double getRPM() {
        return EncoderUtil.toRPM(mRightMotor.getSelectedSensorVelocity(), Constants.Shooter.kFeedbackConfig.getEpr(), Constants.Shooter.kFeedbackConfig.getGearRatio());
    }


    // TODO: Fix shooter current limiting
    public void setCurrent(double amps) {
        //mLeftMotor.configGetStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, amps, 0, 0));
        //mRightMotor.configGetStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, amps, 0, 0));
    }
 
    public int[] getIDs() {
        return new int[] { mLeftMotor.getDeviceID(), mRightMotor.getDeviceID() };
    }

    public DoubleSupplier[] getVoltage() {
        return new DoubleSupplier[] { mLeftMotor::getMotorOutputVoltage, mRightMotor::getMotorOutputVoltage };
    }

    public DoubleSupplier[] getCurrent() {
        return new DoubleSupplier[] { mLeftMotor::getSupplyCurrent, mRightMotor::getSupplyCurrent };
    }

    public BooleanSupplier[] getInverted() {
        return new BooleanSupplier[] { mLeftMotor::getInverted, mRightMotor::getInverted };
    }
}