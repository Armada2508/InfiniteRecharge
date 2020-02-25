package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TransportSubsystem;

public class TransportPower extends CommandBase {
    private double mPower;
    private boolean mElevator;
    private TransportSubsystem mTransportSubsystem;

    public TransportPower(double power, boolean elevator, TransportSubsystem transportSubsystem) {
        mPower = power;
        mElevator = elevator;
        mTransportSubsystem = transportSubsystem;
    }

    @Override
    public void initialize() {
        if(mElevator) {
            mTransportSubsystem.setElevPower(mPower);
        } else {
            mTransportSubsystem.setDiagPower(mPower);
        }
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        if(mElevator) {
            mTransportSubsystem.setElevPower(0);
        } else {
            mTransportSubsystem.setDiagPower(0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}