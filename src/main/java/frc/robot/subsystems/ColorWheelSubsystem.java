package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.MotionMagicConfig;
import frc.lib.config.MotorConfig;
import frc.lib.motion.EncoderUtil;
import frc.robot.Constants;

public class ColorWheelSubsystem extends SubsystemBase {

    private final WPI_TalonSRX m_WOFTalon;

    public ColorWheelSubsystem() {
        m_WOFTalon = new WPI_TalonSRX(Constants.kWOFMotor);
        MotorConfig.configTalon(m_WOFTalon, Constants.kWOFConfig, Constants.kWOFSlot);
        MotionMagicConfig.configTalon(m_WOFTalon, Constants.kWOFMMConfig);
    }

    public void rotate(double rotations) {
        m_WOFTalon.set(ControlMode.Position, EncoderUtil.fromDistance(rotations*Constants.kWOFDiameter*Math.PI, Constants.kWOFEncoderUnitsPerRev, Constants.kWOFGearRatio, Constants.kWOFWheelDiameter));
    }

}