package frc.lib.vision;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

import org.junit.Test;

import frc.lib.util.Util;

public class CameraPoint2dTest {

    @Test
    public void instantiationTest() {
        assertNotNull(new CameraPoint2d(0.0, 0.0));
        assertNotNull(new CameraPoint2d(5.0, 0.0));
        assertNotNull(new CameraPoint2d(0.0, 5.0));
        assertNotNull(new CameraPoint2d(5.0, 5.0));
        assertNotNull(new CameraPoint2d(-5.0, 0.0));
        assertNotNull(new CameraPoint2d(0.0, -5.0));
        assertNotNull(new CameraPoint2d(-5.0, -5.0));
        assertNotNull(new CameraPoint2d(0.0, 0.0, true));
        assertNotNull(new CameraPoint2d(5.0, 0.0, true));
        assertNotNull(new CameraPoint2d(0.0, 5.0, true));
        assertNotNull(new CameraPoint2d(5.0, 5.0, true));
        assertNotNull(new CameraPoint2d(-5.0, 0.0, true));
        assertNotNull(new CameraPoint2d(0.0, -5.0, true));
        assertNotNull(new CameraPoint2d(-5.0, -5.0, true));
        assertNotNull(new CameraPoint2d(0.0, 0.0, false));
        assertNotNull(new CameraPoint2d(5.0, 0.0, false));
        assertNotNull(new CameraPoint2d(0.0, 5.0, false));
        assertNotNull(new CameraPoint2d(5.0, 5.0, false));
        assertNotNull(new CameraPoint2d(-5.0, 0.0, false));
        assertNotNull(new CameraPoint2d(0.0, -5.0, false));
        assertNotNull(new CameraPoint2d(-5.0, -5.0, false));
    }
 
    @Test
    public void setTest() {
        CameraPoint2d point = new CameraPoint2d(0.0, 0.0);
        point.setX(5.0);
        point.setY(-5.0);
        assertEquals(5.0, point.getX(), Util.kEpsilon);
        assertEquals(-5.0, point.getY(), Util.kEpsilon);
        point.setX(-5.0);
        point.setY(5.0);
        assertEquals(-5.0, point.getX(), Util.kEpsilon);
        assertEquals(5.0, point.getY(), Util.kEpsilon);
        point.setX(0.0);
        point.setY(0.0);
        assertEquals(0.0, point.getX(), Util.kEpsilon);
        assertEquals(0.0, point.getY(), Util.kEpsilon);
        point.setX(2.53);
        point.setY(-6.25);
        assertEquals(2.53, point.getX(), Util.kEpsilon);
        assertEquals(-6.25, point.getY(), Util.kEpsilon);
        point.setX(-21.258);
        point.setY(1.355);
        assertEquals(-21.258, point.getX(), Util.kEpsilon);
        assertEquals(1.355, point.getY(), Util.kEpsilon);
    }

    @Test
    public void isAngleTest() {
        CameraPoint2d point = new CameraPoint2d(0.0, 0.0);
        assertEquals(true, point.isAngle());
        point.toPixels(new FOV(60, 40), new Resolution(160, 120));
        assertEquals(false, point.isAngle());
        point.toAngle(new FOV(60, 40), new Resolution(160, 120));
        assertEquals(true, point.isAngle());
    }

    @Test
    public void pixelAngleCalculationTest() {
        CameraPoint2d point = new CameraPoint2d(30.0, 20.0);
        point.toPixels(new FOV(60, 40), new Resolution(320, 240));
        assertEquals(160.0, point.getX(), Util.kEpsilon);
        assertEquals(120.0, point.getY(), Util.kEpsilon);
        point.toAngle(new FOV(60, 40), new Resolution(320, 240));
        assertEquals(30.0, point.getX(), Util.kEpsilon);
        assertEquals(20.0, point.getY(), Util.kEpsilon);
        point.setX(9.7);
        point.setY(-7.3);
        point.toPixels(new FOV(60, 40), new Resolution(320, 240));
        assertEquals(47.3703786332, point.getX(), 1E-9);
        assertEquals(-42.235196967, point.getY(), 1E-9);
        point.toAngle(new FOV(60, 40), new Resolution(320, 240));
        assertEquals(9.7, point.getX(), Util.kEpsilon);
        assertEquals(-7.3, point.getY(), Util.kEpsilon);
        point.setX(-30);
        point.setY(-20);
        point.toPixels(new FOV(60, 40), new Resolution(320, 240));
        assertEquals(-160, point.getX(), Util.kEpsilon);
        assertEquals(-120, point.getY(), Util.kEpsilon);
        point.setX(-500);
        point.setY(-400);
        point.toAngle(new FOV(60, 40), new Resolution(320, 240));
        assertEquals(-30, point.getX(), Util.kEpsilon);
        assertEquals(-20, point.getY(), Util.kEpsilon);
        point.setX(-500);
        point.setY(-400);
        point.toPixels(new FOV(60, 40), new Resolution(320, 240));
        assertEquals(-160, point.getX(), Util.kEpsilon);
        assertEquals(-120, point.getY(), Util.kEpsilon);
        point.setX(500);
        point.setY(400);
        point.toAngle(new FOV(60, 40), new Resolution(320, 240));
        assertEquals(30, point.getX(), Util.kEpsilon);
        assertEquals(20, point.getY(), Util.kEpsilon);
        point.setX(500);
        point.setY(400);
        point.toPixels(new FOV(60, 40), new Resolution(320, 240));
        assertEquals(160, point.getX(), Util.kEpsilon);
        assertEquals(120, point.getY(), Util.kEpsilon);
    }

    

    @Test
    public void globalPixelAngleCalculationTest() {
        CameraPoint2d point = new CameraPoint2d(30.0, 20.0);
        point.config(new FOV(60, 40), new Resolution(320, 240));
        point.toPixels();
        assertEquals(160.0, point.getX(), Util.kEpsilon);
        assertEquals(120.0, point.getY(), Util.kEpsilon);
        point.toAngle();
        assertEquals(30.0, point.getX(), Util.kEpsilon);
        assertEquals(20.0, point.getY(), Util.kEpsilon);
        point.setX(9.7);
        point.setY(-7.3);
        point.toPixels();
        assertEquals(47.3703786332, point.getX(), 1E-9);
        assertEquals(-42.235196967, point.getY(), 1E-9);
        point.toAngle();
        assertEquals(9.7, point.getX(), Util.kEpsilon);
        assertEquals(-7.3, point.getY(), Util.kEpsilon);
        point.setX(-30);
        point.setY(-20);
        point.toPixels();
        assertEquals(-160, point.getX(), Util.kEpsilon);
        assertEquals(-120, point.getY(), Util.kEpsilon);
        point.setX(-500);
        point.setY(-400);
        point.toAngle();
        assertEquals(-30, point.getX(), Util.kEpsilon);
        assertEquals(-20, point.getY(), Util.kEpsilon);
        point.setX(-500);
        point.setY(-400);
        point.toPixels();
        assertEquals(-160, point.getX(), Util.kEpsilon);
        assertEquals(-120, point.getY(), Util.kEpsilon);
        point.setX(500);
        point.setY(400);
        point.toAngle();
        assertEquals(30, point.getX(), Util.kEpsilon);
        assertEquals(20, point.getY(), Util.kEpsilon);
        point.setX(500);
        point.setY(400);
        point.toPixels();
        assertEquals(160, point.getX(), Util.kEpsilon);
        assertEquals(120, point.getY(), Util.kEpsilon);
        point.toAngle();
        point.config(new FOV(80, 60), new Resolution(400, 300));
        point.setX(-40);
        point.setY(-30);
        point.toPixels();
        assertEquals(-200, point.getX(), Util.kEpsilon);
        assertEquals(-150, point.getY(), Util.kEpsilon);
        point.setX(-500);
        point.setY(-400);
        point.toAngle();
        assertEquals(-40, point.getX(), Util.kEpsilon);
        assertEquals(-30, point.getY(), Util.kEpsilon);
        point.setX(-500);
        point.setY(-400);
        point.toPixels();
        assertEquals(-200, point.getX(), Util.kEpsilon);
        assertEquals(-150, point.getY(), Util.kEpsilon);
        point.setX(500);
        point.setY(400);
        point.toAngle();
        assertEquals(40, point.getX(), Util.kEpsilon);
        assertEquals(30, point.getY(), Util.kEpsilon);
        point.setX(500);
        point.setY(400);
        point.toPixels();
        assertEquals(200, point.getX(), Util.kEpsilon);
        assertEquals(150, point.getY(), Util.kEpsilon);
    }
}