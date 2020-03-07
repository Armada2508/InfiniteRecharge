package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TransportSubsystem;

public class TransportPower extends CommandBase {
    private double mPower;
    private boolean mElevator;
    private boolean mDiagonal;
    private TransportSubsystem mTransportSubsystem;

    public TransportPower(TransportSubsystem transportSubsystem, double power, boolean elevator, boolean diagonal) {
        mPower = power;
        mElevator = elevator;
        mDiagonal = diagonal;
        mTransportSubsystem = transportSubsystem;

        // Require TransportSubsystem
        addRequirements(mTransportSubsystem);
    }

    @Override
    public void initialize() {
        if(mElevator) {
            mTransportSubsystem.setElevPower(mPower);
        } 
        if(mDiagonal) {
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
        }
        if(mDiagonal) {
            mTransportSubsystem.setDiagPower(0);
        }
    }

    @Override
    public boolean isFinished() {
        return !mElevator && !mDiagonal;
    }
}