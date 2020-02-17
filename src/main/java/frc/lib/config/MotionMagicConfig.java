package frc.lib.config;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants;

public class MotionMagicConfig {
    private final int m_cruiseVel;
    private final int m_accel;
    private final int m_smoothing;

    /**
     * Creates a new MotionMagicConfig object
     * @param cruiseVelocity The desired cruise velocity
     * @param acceleration The desired maximum acceleration
     * @param smoothing The desired smoothing or S-Curve strength
     */
    public MotionMagicConfig(int cruiseVelocity, int acceleration, int smoothing) {
        m_cruiseVel = cruiseVelocity;
        m_accel = acceleration;
        m_smoothing = smoothing;
    }

    /**
     * @return The desired cruise velocity
     */
    public int getCruiseVelocity() {
        return m_cruiseVel;
    }

    /**
     * @return The desired maximum acceleration
     */
    public int getAcceleration() {
        return m_accel;
    }

    /**
     * @return The desired smoothing or S-Curve strength
     */
    public int getSmoothing() {
        return m_smoothing;
    }

    /**
     * Configure a Talon SRX with the specified MotionMagicConfig
     * @param talon The talon to configure
     * @param config The MotionMagicConfig to apply
     */
    public static void configTalon(TalonSRX talon, MotionMagicConfig config) {
        talon.configMotionCruiseVelocity(config.getCruiseVelocity());
        talon.configMotionAcceleration(config.getAcceleration());
        talon.configMotionSCurveStrength(config.getSmoothing());
		talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, Constants.kMotionMagicPeriod);
		talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, Constants.kMotionMagicPeriod);
    }

    /**
     * Configure a Talon FX with the specified MotionMagicConfig
     * @param talon The talon to configure
     * @param config The MotionMagicConfig to apply
     */
    public static void configTalon(TalonFX talon, MotionMagicConfig config) {
        talon.configMotionCruiseVelocity(config.getCruiseVelocity());
        talon.configMotionAcceleration(config.getAcceleration());
        talon.configMotionSCurveStrength(config.getSmoothing());
		talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, Constants.kMotionMagicPeriod);
		talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, Constants.kMotionMagicPeriod);
    }
}