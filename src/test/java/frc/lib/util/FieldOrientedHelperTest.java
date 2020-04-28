package frc.lib.util;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;

import org.junit.Test;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;


public class FieldOrientedHelperTest {

    @Test
    public void ControllerTest() {
        FieldOrientedHelper controller1 = new FieldOrientedHelper(10.0, new PIDController(0.0, 0.0, 0.0));
        DifferentialDriveWheelSpeeds wheelSpeeds = controller1.calculate(0.0, 1.0, 0);
        assertEquals(wheelSpeeds.leftMetersPerSecond, 1.0, Util.kEpsilon);
        assertEquals(wheelSpeeds.rightMetersPerSecond, 1.0, Util.kEpsilon);
        wheelSpeeds = controller1.calculate(0.0, 5.0, 0);
        assertEquals(wheelSpeeds.leftMetersPerSecond, 5.0, Util.kEpsilon);
        assertEquals(wheelSpeeds.rightMetersPerSecond, 5.0, Util.kEpsilon);
        wheelSpeeds = controller1.calculate(1.5, 2.5, 0);
        assertEquals(wheelSpeeds.leftMetersPerSecond, 2.5, Util.kEpsilon);
        assertEquals(wheelSpeeds.rightMetersPerSecond, 2.5, Util.kEpsilon);
        wheelSpeeds = controller1.calculate(6.28318531, 7.8, 0);
        assertEquals(wheelSpeeds.leftMetersPerSecond, 7.8, Util.kEpsilon);
        assertEquals(wheelSpeeds.rightMetersPerSecond, 7.8, Util.kEpsilon);
        FieldOrientedHelper controller2 = new FieldOrientedHelper(10.0, new PIDController(1.0, 0.0, 0.0));
        wheelSpeeds = controller2.calculate(2.5, 5.0, 0);
        assertNotEquals(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond, Util.kEpsilon);
        assertEquals(wheelSpeeds.leftMetersPerSecond > wheelSpeeds.rightMetersPerSecond, true);
        FieldOrientedHelper controller3 = new FieldOrientedHelper(10.0, new PIDController(1.5, 0.0, 0.0));
        DifferentialDriveWheelSpeeds wheelSpeeds2 = controller3.calculate(2.5, 5.0, 0);
        assertNotEquals(wheelSpeeds.leftMetersPerSecond, wheelSpeeds2.leftMetersPerSecond);
        assertNotEquals(wheelSpeeds.rightMetersPerSecond, wheelSpeeds2.rightMetersPerSecond);
        assertEquals(wheelSpeeds2.rightMetersPerSecond < wheelSpeeds.rightMetersPerSecond, true);
        assertEquals(wheelSpeeds2.leftMetersPerSecond > wheelSpeeds.leftMetersPerSecond, true);
        wheelSpeeds = controller2.calculate(5.0, 5.0, -Math.PI/4);
        assertEquals(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond, Util.kEpsilon);
        wheelSpeeds = controller2.calculate(5.0, 5.0, Math.PI/4);
        assertNotEquals(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond, Util.kEpsilon);
        assertEquals(wheelSpeeds.leftMetersPerSecond, -wheelSpeeds.rightMetersPerSecond, Util.kEpsilon);
     }

}