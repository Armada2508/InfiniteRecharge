/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticsSubsystem extends SubsystemBase {

    private Compressor mCompressor;

    /**
     * Creates a new PneumaticsSubsystem
     */
    public PneumaticsSubsystem() {
        mCompressor = new Compressor();
        mCompressor.setClosedLoopControl(true);
    }


    @Override
    public void periodic() {
        
    }
    
    public boolean getPressureSwitch() {
        return mCompressor.getPressureSwitchValue();
    }

    public boolean getCompressorCurrentTooHighFault() {
        return mCompressor.getCompressorCurrentTooHighFault();
    }

    public boolean getCompressorCurrentTooHighStickyFault() {
        return mCompressor.getCompressorCurrentTooHighStickyFault();
    }

    public boolean getCompressorShortedFault() {
        return mCompressor.getCompressorShortedFault();
    }

    public boolean getCompressorShortedStickyFault() {
        return mCompressor.getCompressorShortedStickyFault();
    }

    public boolean getCompressorNotConnectedFault() {
        return mCompressor.getCompressorNotConnectedFault();
    }

    public boolean getCompressorNotConnectedStickyFault() {
        return mCompressor.getCompressorNotConnectedStickyFault();
    }

}