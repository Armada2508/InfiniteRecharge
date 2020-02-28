package frc.lib.config;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants;

public class MotionMagicConfig {
    private final int mCruiseVel;
    private final int mAccel;
    private final int mSmoothing;

    /**
     * Creates a new MotionMagicConfig object
     * @param cruiseVelocity The desired cruise velocity
     * @param acceleration The desired maximum acceleration
     * @param smoothing The desired smoothing or S-Curve strength
     */
    public MotionMagicConfig(int cruiseVelocity, int acceleration, int smoothing) {
        mCruiseVel = cruiseVelocity;
        mAccel = acceleration;
        mSmoothing = smoothing;
    }

    /**
     * @return The desired cruise velocity
     */
    public int getCruiseVelocity() {
        return mCruiseVel;
    }

    /**
     * @return The desired maximum acceleration
     */
    public int getAcceleration() {
        return mAccel;
    }

    /**
     * @return The desired smoothing or S-Curve strength
     */
    public int getSmoothing() {
        return mSmoothing;
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
		talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, Constants.Config.kMotionMagicPeriod);
		talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, Constants.Config.kMotionMagicPeriod);
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
		talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, Constants.Config.kMotionMagicPeriod);
		talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, Constants.Config.kMotionMagicPeriod);
    }
}