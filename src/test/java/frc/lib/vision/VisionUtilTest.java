package frc.lib.vision;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;

import java.util.ArrayList;

import org.junit.Test;

import frc.lib.util.Util;


public class VisionUtilTest {

    @Test
    public void anglesToPixelsTest() {
        assertEquals(0, VisionUtil.anglesToPixels(0, 60, 320), Util.kEpsilon);
        assertEquals(160, VisionUtil.anglesToPixels(30, 60, 320), Util.kEpsilon);
        assertEquals(-160, VisionUtil.anglesToPixels(-30, 60, 320), Util.kEpsilon);
        assertEquals(160, VisionUtil.anglesToPixels(600, 60, 320), Util.kEpsilon);
        assertEquals(-160, VisionUtil.anglesToPixels(-600, 60, 320), Util.kEpsilon);
        assertEquals(0, VisionUtil.anglesToPixels(0, 50, 100), Util.kEpsilon);
        assertEquals(50, VisionUtil.anglesToPixels(25, 50, 100), Util.kEpsilon);
        assertEquals(-50, VisionUtil.anglesToPixels(-25, 50, 100), Util.kEpsilon);
        assertEquals(50, VisionUtil.anglesToPixels(600, 50, 100), Util.kEpsilon);
        assertEquals(-50, VisionUtil.anglesToPixels(-600, 50, 100), Util.kEpsilon);
        assertEquals(74.2562584, VisionUtil.anglesToPixels(15, 60, 320), 1E-6);
        assertEquals(-74.2562584, VisionUtil.anglesToPixels(-15, 60, 320), 1E-6);
        assertEquals(129.226969, VisionUtil.anglesToPixels(25, 60, 320), 1E-6);
        assertEquals(-129.226969, VisionUtil.anglesToPixels(-25, 60, 320), 1E-6);
    }

    @Test
    public void pixelsToAnglesTest() {
        assertEquals(0, VisionUtil.pixelsToAngles(0, 60, 320), Util.kEpsilon);
        assertEquals(30, VisionUtil.pixelsToAngles(160, 60, 320), Util.kEpsilon);
        assertEquals(-30, VisionUtil.pixelsToAngles(-160, 60, 320), Util.kEpsilon);
        assertEquals(30, VisionUtil.pixelsToAngles(1200, 60, 320), Util.kEpsilon);
        assertEquals(-30, VisionUtil.pixelsToAngles(-1200, 60, 320), Util.kEpsilon);
        assertEquals(0, VisionUtil.pixelsToAngles(0, 50, 100), Util.kEpsilon);
        assertEquals(25, VisionUtil.pixelsToAngles(50, 50, 100), Util.kEpsilon);
        assertEquals(-25, VisionUtil.pixelsToAngles(-50, 50, 100), Util.kEpsilon);
        assertEquals(25, VisionUtil.pixelsToAngles(1200, 50, 100), Util.kEpsilon);
        assertEquals(-25, VisionUtil.pixelsToAngles(-1200, 50, 100), Util.kEpsilon);
        assertEquals(16.1021138, VisionUtil.pixelsToAngles(80, 60, 320), 1E-6);
        assertEquals(-16.1021138, VisionUtil.pixelsToAngles(-80, 60, 320), 1E-6);
        assertEquals(23.4132244, VisionUtil.pixelsToAngles(120, 60, 320), 1E-6);
        assertEquals(-23.4132244, VisionUtil.pixelsToAngles(-120, 60, 320), 1E-6);
        
    }

    @Test
    public void centerPixelsTest() {
        assertEquals(-159.5, VisionUtil.centerPixels(0, 320, false), Util.kEpsilon);
        assertEquals(0.0, VisionUtil.centerPixels(159.5, 320, false), Util.kEpsilon);
        assertEquals(159.5, VisionUtil.centerPixels(319, 320, false), Util.kEpsilon);
        assertEquals(-119.5, VisionUtil.centerPixels(0, 240, false), Util.kEpsilon);
        assertEquals(0.0, VisionUtil.centerPixels(119.5, 240, false), Util.kEpsilon);
        assertEquals(119.5, VisionUtil.centerPixels(239, 240, false), Util.kEpsilon);
        assertEquals(-159.5, VisionUtil.centerPixels(-10, 320, false), Util.kEpsilon);
        assertEquals(159.5, VisionUtil.centerPixels(500, 320, false), Util.kEpsilon);
        assertEquals(-119.5, VisionUtil.centerPixels(-10, 240, false), Util.kEpsilon);
        assertEquals(119.5, VisionUtil.centerPixels(500, 240, false), Util.kEpsilon);
        
        assertEquals(159.5, VisionUtil.centerPixels(0, 320, true), Util.kEpsilon);
        assertEquals(0.0, VisionUtil.centerPixels(159.5, 320, true), Util.kEpsilon);
        assertEquals(-159.5, VisionUtil.centerPixels(319, 320, true), Util.kEpsilon);
        assertEquals(119.5, VisionUtil.centerPixels(0, 240, true), Util.kEpsilon);
        assertEquals(0.0, VisionUtil.centerPixels(119.5, 240, true), Util.kEpsilon);
        assertEquals(-119.5, VisionUtil.centerPixels(239, 240, true), Util.kEpsilon);
        assertEquals(159.5, VisionUtil.centerPixels(-10, 320, true), Util.kEpsilon);
        assertEquals(-159.5, VisionUtil.centerPixels(500, 320, true), Util.kEpsilon);
        assertEquals(119.5, VisionUtil.centerPixels(-10, 240, true), Util.kEpsilon);
        assertEquals(-119.5, VisionUtil.centerPixels(500, 240, true), Util.kEpsilon);

        assertEquals(19.5, VisionUtil.centerPixels(179, 320, false), Util.kEpsilon);
        assertEquals(-139.5, VisionUtil.centerPixels(20, 320, false), Util.kEpsilon);
        assertEquals(-66.5, VisionUtil.centerPixels(53, 240, false), Util.kEpsilon);
        assertEquals(19.5, VisionUtil.centerPixels(139, 240, false), Util.kEpsilon);
        
        assertEquals(135.5, VisionUtil.centerPixels(24, 320, true), Util.kEpsilon);
        assertEquals(-103.5, VisionUtil.centerPixels(263, 320, true), Util.kEpsilon);
        assertEquals(-53.5, VisionUtil.centerPixels(173, 240, true), Util.kEpsilon);
        assertEquals(32.5, VisionUtil.centerPixels(87, 240, true), Util.kEpsilon);

    }

    @Test
    public void parseCornersTest() {
        double[] corners = {
            -159.5, 119.5,
            159.5, -119.5,
            0.0, 0.0,
            120.32, 39.21,
        };
        Resolution res = new Resolution(320, 240);
        FOV fov = new FOV(60, 40);
        CameraPoint2d[] points = VisionUtil.parseCorners(corners, res, fov, 0, 0);
        CameraPoint2d point = new CameraPoint2d(-159.5, 119.5, false);
        point.center(res, false, true);
        point.toAngle(fov, res);
        assertEquals(point.getX(), points[0].getX(), Util.kEpsilon);
        assertEquals(point.getY(), points[0].getY(), Util.kEpsilon);
        point = new CameraPoint2d(159.5, -119.5, false);
        point.center(res, false, true);
        point.toAngle(fov, res);
        assertEquals(point.getX(), points[1].getX(), Util.kEpsilon);
        assertEquals(point.getY(), points[1].getY(), Util.kEpsilon);
        point = new CameraPoint2d(0.0, 0.0, false);
        point.center(res, false, true);
        point.toAngle(fov, res);
        assertEquals(point.getX(), points[2].getX(), Util.kEpsilon);
        assertEquals(point.getY(), points[2].getY(), Util.kEpsilon);
        point = new CameraPoint2d(120.32, 39.21, false);
        point.center(res, false, true);
        point.toAngle(fov, res);
        assertEquals(point.getX(), points[3].getX(), Util.kEpsilon);
        assertEquals(point.getY(), points[3].getY(), Util.kEpsilon);
    }

    @Test
    public void getDistanceWidthTest() {
        Resolution res = new Resolution(320, 240);
        FOV fov = new FOV(60, 40);
        double distance = VisionUtil.getDistanceWidth(4.0, res, fov, 160.0, 0);
        assertEquals(6.92820323027551, distance, Util.kEpsilon);
        distance = VisionUtil.getDistanceWidth(2.164, res, fov, 160.0, 0);
        assertEquals(3.7481579475790507, distance, Util.kEpsilon);
        distance = VisionUtil.getDistanceWidth(4.0, res, fov, 160.0, 20.0);
        assertEquals(8.769716532745722, distance, Util.kEpsilon);
        distance = VisionUtil.getDistanceWidth(8.244, res, fov, 24.0, 0);
        assertEquals(95.19351238398549, distance, Util.kEpsilon);
    }

    @Test
    public void getSkewAngleTest() {
        CameraPoint2d leftCorner = new CameraPoint2d(-5.0, 0.0);
        CameraPoint2d rightCorner = new CameraPoint2d(5.0, 0.0);
        assertEquals(0.0, VisionUtil.getSkewAngle(leftCorner, rightCorner, 4.0, 2.0/Math.tan(Math.toRadians(5))), Util.kEpsilon);
        assertEquals(0.0, VisionUtil.getSkewAngle(leftCorner, rightCorner, 8.0, 4.0/Math.tan(Math.toRadians(5))), Util.kEpsilon);
        leftCorner = new CameraPoint2d(-Math.toDegrees(Math.atan(Math.tan(Math.toRadians(5))*Math.cos(Math.toRadians(45)))), 0.0);
        rightCorner = new CameraPoint2d(Math.toDegrees(Math.atan(Math.tan(Math.toRadians(5))*Math.cos(Math.toRadians(45)))), 2.0);
        assertEquals(45.0, VisionUtil.getSkewAngle(leftCorner, rightCorner, 4.0, 2.0/Math.tan(Math.toRadians(5))), Util.kEpsilon);
        leftCorner = new CameraPoint2d(-Math.toDegrees(Math.atan(Math.tan(Math.toRadians(5))*Math.cos(Math.toRadians(45)))), 2.0);
        rightCorner = new CameraPoint2d(Math.toDegrees(Math.atan(Math.tan(Math.toRadians(5))*Math.cos(Math.toRadians(45)))), 0.0);
        assertEquals(-45.0, VisionUtil.getSkewAngle(leftCorner, rightCorner, 4.0, 2.0/Math.tan(Math.toRadians(5))), Util.kEpsilon);
        leftCorner = new CameraPoint2d(-10.0, 2.0);
        rightCorner = new CameraPoint2d(10.0, 0.0);
        assertEquals(0.0, VisionUtil.getSkewAngle(leftCorner, rightCorner, 4.0, 2.0/Math.tan(Math.toRadians(10))), Util.kEpsilon);
        
    }

    @Test
    public void getTopCornersTest() {
        CameraPoint2d[] test1 = {
            new CameraPoint2d(5.0, 5.0),
            new CameraPoint2d(-5.0, 5.0),
            new CameraPoint2d(5.0, -5.0),
            new CameraPoint2d(-5.0, -5.0),
        };
        CameraPoint2d[] result1 = {
            new CameraPoint2d(-5.0, 5.0),
            new CameraPoint2d(5.0, 5.0)
        };
        assertArrayEquals(result1, VisionUtil.getTopCorners(test1));
        CameraPoint2d[] test2 = {
            new CameraPoint2d(12.642, 9.145),
            new CameraPoint2d(-23.532, 17.483),
            new CameraPoint2d(17.526, -27.264),
            new CameraPoint2d(-7.924, -2.826),
            new CameraPoint2d(27.135, -15.282),
            new CameraPoint2d(-11.524, -4.332),
        };
        CameraPoint2d[] result2 = {
            new CameraPoint2d(-23.532, 17.483),
            new CameraPoint2d(12.642, 9.145),
        };
        assertArrayEquals(result2, VisionUtil.getTopCorners(test2));
    }

}