package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.MotionMagicConfig;
import frc.lib.config.MotorConfig;
import frc.lib.motion.EncoderUtil;
import frc.robot.Constants;

public class ColorWheelSubsystem extends SubsystemBase {

    private final WPI_TalonSRX mWOFTalon;

    public ColorWheelSubsystem() {
        mWOFTalon = new WPI_TalonSRX(Constants.kWOFMotor);
        MotorConfig.configTalon(mWOFTalon, Constants.kWOFConfig, Constants.kWOFSlot);
        MotionMagicConfig.configTalon(mWOFTalon, Constants.kWOFMMConfig);
    }

    public void rotate(double rotations) {
        mWOFTalon.set(ControlMode.Position, EncoderUtil.fromDistance(rotations*Constants.kWOFDiameter*Math.PI, Constants.kWOFEncoderUnitsPerRev, Constants.kWOFGearRatio, Constants.kWOFWheelDiameter));
    }

}