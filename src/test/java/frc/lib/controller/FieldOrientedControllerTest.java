package frc.lib.controller;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import frc.lib.util.Util;


public class FieldOrientedControllerTest {

    @Test
    public void ControllerTest() {
        FieldOrientedController controller1 = new FieldOrientedController(10.0, new PIDController(0.0, 0.0, 0.0));
        DifferentialDriveWheelSpeeds wheelSpeeds1 = controller1.calculate(0.0, 1.0, 0);
        assertEquals(wheelSpeeds1.leftMetersPerSecond, 1.0, Util.kEpsilon);
        assertEquals(wheelSpeeds1.rightMetersPerSecond, 1.0, Util.kEpsilon);
        DifferentialDriveWheelSpeeds wheelSpeeds2 = controller1.calculate(0.0, 5.0, 0);
        assertEquals(wheelSpeeds2.leftMetersPerSecond, 5.0, Util.kEpsilon);
        assertEquals(wheelSpeeds2.rightMetersPerSecond, 5.0, Util.kEpsilon);
    }

}