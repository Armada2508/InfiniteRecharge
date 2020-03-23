package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

    private WPI_TalonSRX mIntakeTalon;

    public IntakeSubsystem(int talonID, boolean inverted) {
        mIntakeTalon = new WPI_TalonSRX(talonID);
        mIntakeTalon.setInverted(inverted);
        mIntakeTalon.configOpenloopRamp(Constants.Intake.kRamp);
    }

    @Override
    public void periodic() {
        
    }

    public void set(double power) {
        mIntakeTalon.set(ControlMode.PercentOutput, power);
    }
}