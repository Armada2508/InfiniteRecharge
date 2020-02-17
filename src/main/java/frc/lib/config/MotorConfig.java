package frc.lib.config;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class MotorConfig {
    private final double m_p;
    private final double m_i;
    private final double m_d;
    private final double m_f;
    private final double m_mia;
    private final int m_maxCurrent;
    private final FeedbackDevice m_encoder;
    private final double m_deadband;

    /**
     * Creates a new MotorConfig object to store motor controller configuration settings
     * @param p The propritional constant
     * @param i The integral constant
     * @param d The derivitive constant
     * @param f The feed-forward constant
     * @param mia The maximum integral accumulator constant
     * @param maxCurrent The maximum current the motor will draw
     * @param encoder The type of encoder the motor uses
     * @param deadband The deadband(0.0-1.0) to be applied to the motor controller
     */
    public MotorConfig(double p, double i, double d, double f, double mia, int maxCurrent, FeedbackDevice encoder, double deadband) {
        m_p = p;
        m_i = i;
        m_d = d;
        m_f = f;
        m_mia = mia;
        m_maxCurrent = maxCurrent;
        m_encoder = encoder;
        m_deadband = deadband;
    }

    /**
     * @return The proportional constant
     */
    public double getP() {
        return m_p;
    }

    /**
     * @return The integral constant
     */
    public double getI() {
        return m_i;
    }

    /**
     * @return The derivitive constant
     */
    public double getD() {
        return m_d;
    }

    /**
     * @return The feed-forward constant
     */
    public double getF() {
        return m_f;
    }

    /**
     * @return The max integral accumulator constant
     */
    public double getMIA() {
        return m_mia;
    }

    /**
     * @return The max current the motor will draw
     */
    public int getMaxCurrent() {
        return m_maxCurrent;
    }

    /**
     * @return The encoder type
     */
    public FeedbackDevice getEncoder() {
        return m_encoder;
    }

    /**
     * @return The size of the deadband
     */
    public double getDeadband() {
        return m_deadband;
    }
    
    
    /**
     * Configure a Talon SRX with the specified MotorConfig
     * @param talon The Talon SRX to be configure
     * @param config The configuration to apply
     * @param slot The PID slot to use
     */
    public static void configTalon(TalonSRX talon, MotorConfig config, int slot) {
        talon.configFactoryDefault();
        talon.selectProfileSlot(slot, 0);
        talon.config_kP(slot, config.getP());
        talon.config_kI(slot, config.getI());
        talon.config_kD(slot, config.getD());
        talon.config_kF(slot, config.getF());
        talon.configMaxIntegralAccumulator(slot, config.getMIA());
        talon.configContinuousCurrentLimit(slot, config.getMaxCurrent());
        talon.enableCurrentLimit(true);
        talon.configSelectedFeedbackSensor(config.getEncoder());
        talon.configNeutralDeadband(config.getDeadband());
		talon.configNominalOutputForward(0);
		talon.configNominalOutputReverse(0);
		talon.configPeakOutputForward(1);
		talon.configPeakOutputReverse(-1);
    }
    
    /**
     * Configure a Talon FX with the specified MotorConfig
     * @param talon The Talon FX to be configure
     * @param config The MotorConfig to apply
     * @param slot The PID slot to use
     */
    public static void configTalon(TalonFX talon, MotorConfig config, int slot) {
        talon.configFactoryDefault();
        talon.selectProfileSlot(slot, 0);
        talon.config_kP(slot, config.getP());
        talon.config_kI(slot, config.getI());
        talon.config_kD(slot, config.getD());
        talon.config_kF(slot, config.getF());
        talon.configMaxIntegralAccumulator(slot, config.getMIA());
        talon.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, config.getMaxCurrent(), 0, 0));
        talon.configSelectedFeedbackSensor(config.getEncoder());
        talon.configNeutralDeadband(config.getDeadband());
		talon.configNominalOutputForward(0);
		talon.configNominalOutputReverse(0);
		talon.configPeakOutputForward(1);
		talon.configPeakOutputReverse(-1);
    }
}