package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TransportSubsystem;

public class TransportPower extends CommandBase {
    private double mPower;
    private TransportSubsystem mTransportSubsystem;

    public TransportPower(TransportSubsystem transportSubsystem, double power) {
        mPower = power;
        mTransportSubsystem = transportSubsystem;

        // Require TransportSubsystem
        addRequirements(mTransportSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        mTransportSubsystem.setPower(mPower);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}