package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.MotionMagicConfig;
import frc.lib.config.MotorConfig;
import frc.lib.motion.EncoderUtil;
import frc.robot.Constants;

public class ColorWheelSubsystem extends SubsystemBase {

    private final WPI_TalonSRX mWOFTalon;

    public ColorWheelSubsystem() {
        mWOFTalon = new WPI_TalonSRX(Constants.WOF.kWOFMotor);
        MotorConfig.configTalon(mWOFTalon, Constants.WOF.kWOFConfig, Constants.WOF.kWOFSlot);
        MotionMagicConfig.configTalon(mWOFTalon, Constants.WOF.kWOFMMConfig);
    }

    public void rotate(double rotations) {
        mWOFTalon.set(ControlMode.MotionMagic, EncoderUtil.fromDistance(rotations*Constants.WOF.kWOFDiameter*Math.PI, Constants.WOF.kWOFEncoderUnitsPerRev, Constants.WOF.kWOFGearRatio, Constants.WOF.kWOFWheelDiameter));
    }

    public void reset() {
        mWOFTalon.setSelectedSensorPosition(0);
    }

    public double getRotations() {
        return EncoderUtil.toDistance(mWOFTalon.getSelectedSensorPosition(), Constants.WOF.kWOFEncoderUnitsPerRev, Constants.WOF.kWOFGearRatio, Constants.WOF.kWOFWheelDiameter)/(Constants.WOF.kWOFDiameter*Math.PI);
    }

    public double getRPM() {
        return EncoderUtil.toRPM(mWOFTalon.getSelectedSensorVelocity(), Constants.WOF.kWOFEncoderUnitsPerRev, Constants.WOF.kWOFGearRatio, Constants.Drive.kVelSampleTime);
    }

    public char getColor() {
        return DriverStation.getInstance().getGameSpecificMessage().charAt(0);
    }

    public String getColorString() {
        switch(DriverStation.getInstance().getGameSpecificMessage().charAt(0))
        {
            case 'B' :
                return "blue";
            case 'G' :
                return "green";
            case 'R' :
                return "red";
            case 'Y' :
                return "yellow";
            default :
                return "";
        }
    }

}