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

    private Compressor m_compressor;

    /**
     * Creates a new PneumaticsSubsystem
     */
    public PneumaticsSubsystem() {
        m_compressor = new Compressor();
        m_compressor.setClosedLoopControl(true);
    }


    @Override
    public void periodic() {
        
    }

}