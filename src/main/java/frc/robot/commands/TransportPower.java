package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TransportSubsystem;

public class TransportPower extends CommandBase {
    private double m_power;
    private boolean m_elevator;
    private TransportSubsystem m_transportSubsystem;

    public TransportPower(double power, boolean elevator, TransportSubsystem transportSubsystem) {
        m_power = power;
        m_elevator = elevator;
        m_transportSubsystem = transportSubsystem;
    }

    @Override
    public void initialize() {
        if(m_elevator) {
            m_transportSubsystem.setElevPower(m_power);
        } else {
            m_transportSubsystem.setDiagPower(m_power);
        }
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        if(m_elevator) {
            m_transportSubsystem.setElevPower(0);
        } else {
            m_transportSubsystem.setDiagPower(0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}