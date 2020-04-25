package frc.lib.util;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.Arrays;

import org.junit.Test;

import edu.wpi.first.wpilibj.geometry.Rotation2d;

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

    @Test
    public void epsilonEqualsTest() {
        assertEquals(Util.epsilonEquals(0.0, 1e-14), true);
        assertEquals(Util.epsilonEquals(0.0, 1e-13), true);
        assertEquals(Util.epsilonEquals(0.0, 1e-12), true);
        assertEquals(Util.epsilonEquals(0.0, 1e-11), false);
        assertEquals(Util.epsilonEquals(0.0, -1e-14), true);
        assertEquals(Util.epsilonEquals(0.0, -1e-13), true);
        assertEquals(Util.epsilonEquals(0.0, -1e-12), true);
        assertEquals(Util.epsilonEquals(0.0, -1e-11), false);
        assertEquals(Util.epsilonEquals(1e-14, 0.0), true);
        assertEquals(Util.epsilonEquals(1e-13, 0.0), true);
        assertEquals(Util.epsilonEquals(1e-12, 0.0), true);
        assertEquals(Util.epsilonEquals(1e-11, 0.0), false);
        assertEquals(Util.epsilonEquals(-1e-14, 0.0), true);
        assertEquals(Util.epsilonEquals(-1e-13, 0.0), true);
        assertEquals(Util.epsilonEquals(-1e-12, 0.0), true);
        assertEquals(Util.epsilonEquals(-1e-11, 0.0), false);
    }

    @Test
    public void allCloseToTest() {
        assertEquals(Util.allCloseTo(Arrays.asList(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0), 1.0), true);
        assertEquals(Util.allCloseTo(Arrays.asList(5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0), 5.0), true);
        assertEquals(Util.allCloseTo(Arrays.asList(-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0), -1.0), true);
        assertEquals(Util.allCloseTo(Arrays.asList(-5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0), -5.0), true);
        assertEquals(Util.allCloseTo(Arrays.asList(1.0, 1.0, 1.0, 1.0, 1.2, 1.0, 1.0), 1.0), false);
        assertEquals(Util.allCloseTo(Arrays.asList(5.0, 5.0, 5.4, 5.0, 5.0, 5.0, 5.0), 5.0), false);
        assertEquals(Util.allCloseTo(Arrays.asList(-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.1), -1.0), false);
        assertEquals(Util.allCloseTo(Arrays.asList(-5.8, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0), -5.0), false);
    }

    @Test
    public void boundedAngleTest() {
        assertEquals(Util.boundedAngle(Math.PI/2.0, false), Math.PI/2.0, Util.kEpsilon);
        assertEquals(Util.boundedAngle(Math.PI, false), Math.PI, Util.kEpsilon);
        assertEquals(Util.boundedAngle(-Math.PI/2.0, false), -Math.PI/2.0, Util.kEpsilon);
        assertEquals(Util.boundedAngle(2*Math.PI, false), 0.0, Util.kEpsilon);
        assertEquals(Util.boundedAngle(-3*Math.PI/2.0, false), Math.PI/2.0, Util.kEpsilon);
        assertEquals(Util.boundedAngle(3*Math.PI/2.0, false), -Math.PI/2.0, Util.kEpsilon);
        assertEquals(Util.boundedAngle(3*Math.PI, false), Math.PI, Util.kEpsilon);
        assertEquals(Util.boundedAngle(-3*Math.PI, false), Math.PI, Util.kEpsilon);
        assertEquals(Util.boundedAngle(5*Math.PI, false), Math.PI, Util.kEpsilon);
        assertEquals(Util.boundedAngle(-5*Math.PI, false), Math.PI, Util.kEpsilon);
        
        assertEquals(Util.boundedAngle(Math.PI/2.0, true), Math.PI/2.0, Util.kEpsilon);
        assertEquals(Util.boundedAngle(Math.PI, true), Math.PI, Util.kEpsilon);
        assertEquals(Util.boundedAngle(-Math.PI, true), Math.PI, Util.kEpsilon);
        assertEquals(Util.boundedAngle(-Math.PI/2.0, true), 3.0*Math.PI/2.0, Util.kEpsilon);
        assertEquals(Util.boundedAngle(2*Math.PI, true), 0.0, Util.kEpsilon);
        assertEquals(Util.boundedAngle(-2*Math.PI, true), 0.0, Util.kEpsilon);
        assertEquals(Util.boundedAngle(3*Math.PI, true), Math.PI, Util.kEpsilon);
        assertEquals(Util.boundedAngle(-3*Math.PI, true), Math.PI, Util.kEpsilon);
        
        assertEquals(Util.boundedAngle(new Rotation2d(Math.PI/2.0), false).getRadians(), Math.PI/2.0, Util.kEpsilon);
        assertEquals(Util.boundedAngle(new Rotation2d(Math.PI), true).getRadians(), Math.PI, Util.kEpsilon);
        assertEquals(Util.boundedAngle(new Rotation2d(-Math.PI), false).getRadians(), Math.PI, Util.kEpsilon);
        assertEquals(Util.boundedAngle(new Rotation2d(2*Math.PI), true).getRadians(), 0.0, Util.kEpsilon);
        assertEquals(Util.boundedAngle(new Rotation2d(-2*Math.PI), false).getRadians(), 0.0, Util.kEpsilon);
        assertEquals(Util.boundedAngle(new Rotation2d(3*Math.PI), true).getRadians(), Math.PI, Util.kEpsilon);
        assertEquals(Util.boundedAngle(new Rotation2d(-3*Math.PI), false).getRadians(), Math.PI, Util.kEpsilon);
    }

    

    @Test
    public void boundedAngleDegreesTest() {
        assertEquals(Util.boundedAngleDegrees(90, true), 90, Util.kEpsilon);
        assertEquals(Util.boundedAngleDegrees(180, true), 180, Util.kEpsilon);
        assertEquals(Util.boundedAngleDegrees(-180, true), 180, Util.kEpsilon);
        assertEquals(Util.boundedAngleDegrees(-90, true), 270, Util.kEpsilon);
        assertEquals(Util.boundedAngleDegrees(-45, true), 315, Util.kEpsilon);
        assertEquals(Util.boundedAngleDegrees(540, true), 180, Util.kEpsilon);
        assertEquals(Util.boundedAngleDegrees(-540, true), 180, Util.kEpsilon);

        
        assertEquals(Util.boundedAngleDegrees(90, false), 90, Util.kEpsilon);
        assertEquals(Util.boundedAngleDegrees(180, false), 180, Util.kEpsilon);
        assertEquals(Util.boundedAngleDegrees(-180, false), 180, Util.kEpsilon);
        assertEquals(Util.boundedAngleDegrees(-90, false), -90, Util.kEpsilon);
        assertEquals(Util.boundedAngleDegrees(-45, false), -45, Util.kEpsilon);
        assertEquals(Util.boundedAngleDegrees(540, false), 180, Util.kEpsilon);
        assertEquals(Util.boundedAngleDegrees(-540, false), 180, Util.kEpsilon);
    }
}