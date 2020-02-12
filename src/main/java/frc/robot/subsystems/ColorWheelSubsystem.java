package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ColorWheelSubsystem extends SubsystemBase {
    public WPI_TalonSRX m_colorWheelMotor;

    public ColorWheelSubsystem(int talonID) {
        m_colorWheelMotor = new WPI_TalonSRX(talonID);
    }

    public void rotate(double rotations) {
        m_colorWheelMotor.set(ControlMode.Position, (rotations*Constants.kColorWheelDiameter*Math.PI)/(Constants.kEncoderUnitsPerRev*Constants.kGearRatio*Math.PI*Constants.kWOFStealthWheelDiameter));
    }
}