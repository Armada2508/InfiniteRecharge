package frc.lib.input;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import frc.lib.util.Util;


public class JoystickUtilTest {

    @Test
    public void deadbandTest() {
        // Test Deadband Method
        assertEquals(0, JoystickUtil.deadband(0.025, 0.1), Util.kEpsilon);
        assertEquals(0.1, JoystickUtil.deadband(0.1, 0.01), Util.kEpsilon);
        assertEquals(0, JoystickUtil.deadband(0.1, 0.1), Util.kEpsilon);
        assertEquals(0, JoystickUtil.deadband(Math.PI, 3.15), Util.kEpsilon);
        assertEquals(Math.PI, JoystickUtil.deadband(Math.PI, 3.14), Util.kEpsilon);
    }

}