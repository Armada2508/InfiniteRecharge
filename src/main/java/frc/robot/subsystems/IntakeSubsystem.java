package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    public WPI_TalonSRX mIntakeTalon;

    public IntakeSubsystem(int talonID, boolean inverted) {
        mIntakeTalon = new WPI_TalonSRX(talonID);
        mIntakeTalon.setInverted(inverted);
    }

    @Override
    public void periodic() {
        
    }

    public void set(double power) {
        mIntakeTalon.set(ControlMode.PercentOutput, power);
    }
}