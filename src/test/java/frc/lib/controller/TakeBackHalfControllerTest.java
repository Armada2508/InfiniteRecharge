package frc.lib.controller;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import frc.lib.util.Util;

public class TakeBackHalfControllerTest {
    @Test
    public void ControllerTest() {
        // Test the controller
        TakeBackHalfController controller = new TakeBackHalfController(0.5);
        controller.setSetpoint(1.0);
        // If we are at the setpoint the control effort should be 0
        assertEquals(controller.calculate(1.0), 0.0, Util.kEpsilon);
        // If we have an error of -1 and change the sign of the error
        // we should have a control effort of -0.005(-1*0.02*0.5 / 2)
        assertEquals(controller.calculate(2.0), -0.005, Util.kEpsilon);
        // That should increase by -0.01 over time(-1*0.5*0.02)
        assertEquals(controller.calculate(2.0), -0.015, Util.kEpsilon);
        assertEquals(controller.calculate(2.0), -0.025, Util.kEpsilon);
        assertEquals(controller.calculate(2.0), -0.035, Util.kEpsilon);
        // If we change the sign again it should average the values that
        // changed the sign(-0.005 and -0.035)
        assertEquals(controller.calculate(1.0), -0.02, Util.kEpsilon);
        // Test Reset
        controller.reset();
        controller.setSetpoint(1.0);
        assertEquals(controller.calculate(1.0), 0.0, Util.kEpsilon);
    }
}