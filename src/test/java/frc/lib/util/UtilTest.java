package frc.lib.util;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;

import org.junit.Test;

public class UtilTest {

    @Test
    public void inRangeMagnitudeTest() {
        assertEquals(true, Util.inRange(0.0, 1.0));
        assertEquals(true, Util.inRange(0.5, 1.0));
        assertEquals(false, Util.inRange(1.5, 1.0));
        assertEquals(false, Util.inRange(0.0, -1.0));
        assertEquals(true, Util.inRange(-0.5, 1.0));
        assertEquals(false, Util.inRange(-1.5, 1.0));
        assertEquals(false, Util.inRange(-1.0, 1.0));
        assertEquals(false, Util.inRange(1.0, 1.0));
        assertEquals(true, Util.inRange(0.0, 0.75));
        assertEquals(true, Util.inRange(0.5, 0.75));
        assertEquals(false, Util.inRange(1.5, 0.75));
        assertEquals(false, Util.inRange(0.0, -0.75));
        assertEquals(true, Util.inRange(-0.5, 0.75));
        assertEquals(false, Util.inRange(-1.5, 0.75));
        assertEquals(false, Util.inRange(-0.75, 0.75));
        assertEquals(false, Util.inRange(0.75, 0.75));
    }

    @Test
    public void inRangeTest() {
        assertEquals(true, Util.inRange(0.0, -0.5, 0.5));
        assertEquals(false, Util.inRange(-0.5, -0.5, 0.5));
        assertEquals(false, Util.inRange(0.5, -0.5, 0.5));
        assertEquals(false, Util.inRange(Math.PI, 0.5, Math.PI));
        assertEquals(true, Util.inRange(Math.PI, -0.5, 3.15));
        assertEquals(true, Util.inRange(Math.PI, -0.1, 5));
        assertEquals(false, Util.inRange(Math.PI, 5, Math.PI));
    }

    @Test
    public void lerpTest() {
        assertEquals(0.0, Util.lerp(0.0, 1.0, 0.0), Util.kEpsilon);
        assertEquals(0.5, Util.lerp(0.0, 1.0, 0.5), Util.kEpsilon);
        assertEquals(1.0, Util.lerp(0.0, 1.0, 1.0), Util.kEpsilon);
        assertEquals(0.32523531562, Util.lerp(0.0, 1.0, 0.32523531562), Util.kEpsilon);
        assertEquals(0.0, Util.lerp(0.0, 1.0, -1.0), Util.kEpsilon);
        assertEquals(1.0, Util.lerp(0.0, 1.0, 20.0), Util.kEpsilon);
        assertEquals(0.0, Util.lerp(0.0, 3.5, 0.0), Util.kEpsilon);
        assertEquals(1.05, Util.lerp(0.0, 2.1, 0.5), Util.kEpsilon);
        assertEquals(8.4, Util.lerp(0.0, 8.4, 1.0), Util.kEpsilon);
        assertEquals(1.054625, Util.lerp(0.0, 3.25, 0.3245), Util.kEpsilon);
        assertEquals(1.172016, Util.lerp(0.0, 2.16, 0.5426), Util.kEpsilon);
        assertEquals(1.0, Util.lerp(1.0, 2.0, 0.0), Util.kEpsilon);
        assertEquals(1.5, Util.lerp(1.0, 2.0, 0.5), Util.kEpsilon);
        assertEquals(2.0, Util.lerp(1.0, 2.0, 1.0), Util.kEpsilon);
        assertEquals(1.32523531562, Util.lerp(1.0, 2.0, 0.32523531562), Util.kEpsilon);
        assertEquals(4.262646, Util.lerp(4.262646, 8.24246, -1.0), Util.kEpsilon);
        assertEquals(84.983747, Util.lerp(28.8362847, 84.983747, 20.0), Util.kEpsilon);
        assertEquals(4.05, Util.lerp(3.5, 4.6, 0.5), Util.kEpsilon);
        assertEquals(8.4, Util.lerp(2.3, 8.4, 1.0), Util.kEpsilon);
        assertEquals(1.254625, Util.lerp(0.2, 3.45, 0.3245), Util.kEpsilon);
        assertEquals(1.672016, Util.lerp(0.5, 2.66, 0.5426), Util.kEpsilon);
    }

    @Test
    public void joinStringsTest() {
        ArrayList<String> strings = new ArrayList<>();
        strings.add("This");
        strings.add("is");
        strings.add("a");
        strings.add("unit");
        strings.add("test!");
        assertEquals("This is a unit test!", Util.joinStrings(" ", strings));
        assertEquals("This;is;a;unit;test!", Util.joinStrings(";", strings));
    }

}