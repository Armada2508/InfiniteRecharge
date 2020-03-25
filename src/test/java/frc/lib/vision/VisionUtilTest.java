package frc.lib.vision;

import static org.junit.Assert.assertEquals;

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

}