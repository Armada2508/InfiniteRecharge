package frc.lib.motion;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import frc.lib.util.Util;


public class EncoderUtilTest {
    
    @Test
    public void toDistanceTest() {
        // Test Converting from Encoder Units to Distance
        assertEquals(Math.PI, EncoderUtil.toDistance(4096, 4096, 1, 1), Util.kEpsilon);
        assertEquals(2*Math.PI, EncoderUtil.toDistance(8192, 4096, 1, 1), Util.kEpsilon);
        assertEquals(Math.PI, EncoderUtil.toDistance(8192, 4096, 2, 1), Util.kEpsilon);
        assertEquals(Math.PI, EncoderUtil.toDistance(4096, 8192, 1, 2), Util.kEpsilon);
        assertEquals(Math.PI*2000/8192, EncoderUtil.toDistance(2000, 8192, 1, 1), Util.kEpsilon);
        assertEquals(Math.PI, EncoderUtil.toDistance(1024, 1024, 1, 1), Util.kEpsilon);
        assertEquals(2*Math.PI, EncoderUtil.toDistance(2048, 1024, 1, 1), Util.kEpsilon);
        assertEquals(Math.PI, EncoderUtil.toDistance(2048, 1024, 2, 1), Util.kEpsilon);
        assertEquals(Math.PI, EncoderUtil.toDistance(1024, 2048, 1, 2), Util.kEpsilon);
        assertEquals(Math.PI*2000/1024, EncoderUtil.toDistance(2000, 1024, 1, 1), Util.kEpsilon);
    }

    @Test
    public void fromDistanceTest() {
        // Test Converting from Distance to Encoder Units
        assertEquals(4096, EncoderUtil.fromDistance(Math.PI, 4096, 1, 1), Util.kEpsilon);
        assertEquals(8192, EncoderUtil.fromDistance(Math.PI, 4096, 2, 1), Util.kEpsilon);
        assertEquals(4096, EncoderUtil.fromDistance(Math.PI, 4096, 2, 2), Util.kEpsilon);
        assertEquals(4096*5/Math.PI, EncoderUtil.fromDistance(5, 4096, 1, 1), Util.kEpsilon);
        assertEquals(1024, EncoderUtil.fromDistance(Math.PI, 1024, 1, 1), Util.kEpsilon);
        assertEquals(2048, EncoderUtil.fromDistance(Math.PI, 1024, 2, 1), Util.kEpsilon);
        assertEquals(1024, EncoderUtil.fromDistance(Math.PI, 1024, 2, 2), Util.kEpsilon);
        assertEquals(1024*5/Math.PI, EncoderUtil.fromDistance(5, 1024, 1, 1), Util.kEpsilon);
    }

    
    @Test
    public void toVelocityTest() {
        // Test Converting from Encoder Unit Velocity to Velocity
        assertEquals(10*Math.PI, EncoderUtil.toVelocity(4096, 4096, 1, 1), Util.kEpsilon);
        assertEquals(20*Math.PI, EncoderUtil.toVelocity(8192, 4096, 1, 1), Util.kEpsilon);
        assertEquals(10*Math.PI, EncoderUtil.toVelocity(8192, 4096, 2, 1), Util.kEpsilon);
        assertEquals(10*Math.PI, EncoderUtil.toVelocity(4096, 8192, 1, 2), Util.kEpsilon);
        assertEquals(10*Math.PI*2000/8192, EncoderUtil.toVelocity(2000, 8192, 1, 1), Util.kEpsilon);
        assertEquals(10*Math.PI, EncoderUtil.toVelocity(1024, 1024, 1, 1), Util.kEpsilon);
        assertEquals(20*Math.PI, EncoderUtil.toVelocity(2048, 1024, 1, 1), Util.kEpsilon);
        assertEquals(10*Math.PI, EncoderUtil.toVelocity(2048, 1024, 2, 1), Util.kEpsilon);
        assertEquals(10*Math.PI, EncoderUtil.toVelocity(1024, 2048, 1, 2), Util.kEpsilon);
        assertEquals(10*Math.PI*2000/1024, EncoderUtil.toVelocity(2000, 1024, 1, 1), Util.kEpsilon);
        
        assertEquals(2*Math.PI, EncoderUtil.toVelocity(4096, 4096, 1, 1, 0.5), Util.kEpsilon);
        assertEquals(4*Math.PI, EncoderUtil.toVelocity(8192, 4096, 1, 1, 0.5), Util.kEpsilon);
        assertEquals(2*Math.PI, EncoderUtil.toVelocity(8192, 4096, 2, 1, 0.5), Util.kEpsilon);
        assertEquals(2*Math.PI, EncoderUtil.toVelocity(4096, 8192, 1, 2, 0.5), Util.kEpsilon);
        assertEquals(2*Math.PI*2000/8192, EncoderUtil.toVelocity(2000, 8192, 1, 1, 0.5), Util.kEpsilon);
        assertEquals(2*Math.PI, EncoderUtil.toVelocity(1024, 1024, 1, 1, 0.5), Util.kEpsilon);
        assertEquals(4*Math.PI, EncoderUtil.toVelocity(2048, 1024, 1, 1, 0.5), Util.kEpsilon);
        assertEquals(2*Math.PI, EncoderUtil.toVelocity(2048, 1024, 2, 1, 0.5), Util.kEpsilon);
        assertEquals(2*Math.PI, EncoderUtil.toVelocity(1024, 2048, 1, 2, 0.5), Util.kEpsilon);
        assertEquals(2*Math.PI*2000/1024, EncoderUtil.toVelocity(2000, 1024, 1, 1, 0.5), Util.kEpsilon);
        
        assertEquals(10*Math.PI, EncoderUtil.toVelocity(4096, 4096, 1, 1, 0.1), Util.kEpsilon);
        assertEquals(20*Math.PI, EncoderUtil.toVelocity(8192, 4096, 1, 1, 0.1), Util.kEpsilon);
        assertEquals(10*Math.PI, EncoderUtil.toVelocity(8192, 4096, 2, 1, 0.1), Util.kEpsilon);
        assertEquals(10*Math.PI, EncoderUtil.toVelocity(4096, 8192, 1, 2, 0.1), Util.kEpsilon);
        assertEquals(10*Math.PI*2000/8192, EncoderUtil.toVelocity(2000, 8192, 1, 1, 0.1), Util.kEpsilon);
        assertEquals(10*Math.PI, EncoderUtil.toVelocity(1024, 1024, 1, 1, 0.1), Util.kEpsilon);
        assertEquals(20*Math.PI, EncoderUtil.toVelocity(2048, 1024, 1, 1, 0.1), Util.kEpsilon);
        assertEquals(10*Math.PI, EncoderUtil.toVelocity(2048, 1024, 2, 1, 0.1), Util.kEpsilon);
        assertEquals(10*Math.PI, EncoderUtil.toVelocity(1024, 2048, 1, 2, 0.1), Util.kEpsilon);
        assertEquals(10*Math.PI*2000/1024, EncoderUtil.toVelocity(2000, 1024, 1, 1, 0.1), Util.kEpsilon);
    }

    @Test
    public void fromVelocityTest() {
        // Test Converting from Velocity to Encoder Unit Velocity
        assertEquals(409.6, EncoderUtil.fromVelocity(Math.PI, 4096, 1, 1), Util.kEpsilon);
        assertEquals(819.2, EncoderUtil.fromVelocity(Math.PI, 4096, 2, 1), Util.kEpsilon);
        assertEquals(409.6, EncoderUtil.fromVelocity(Math.PI, 4096, 2, 2), Util.kEpsilon);
        assertEquals(409.6*5/Math.PI, EncoderUtil.fromVelocity(5, 4096, 1, 1), Util.kEpsilon);
        assertEquals(102.4, EncoderUtil.fromVelocity(Math.PI, 1024, 1, 1), Util.kEpsilon);
        assertEquals(204.8, EncoderUtil.fromVelocity(Math.PI, 1024, 2, 1), Util.kEpsilon);
        assertEquals(102.4, EncoderUtil.fromVelocity(Math.PI, 1024, 2, 2), Util.kEpsilon);
        assertEquals(102.4*5/Math.PI, EncoderUtil.fromVelocity(5, 1024, 1, 1), Util.kEpsilon);
        
        assertEquals((4096/2.0), EncoderUtil.fromVelocity(Math.PI, 4096, 1, 1, 0.5), Util.kEpsilon);
        assertEquals((8192/2.0), EncoderUtil.fromVelocity(Math.PI, 4096, 2, 1, 0.5), Util.kEpsilon);
        assertEquals((4096/2.0), EncoderUtil.fromVelocity(Math.PI, 4096, 2, 2, 0.5), Util.kEpsilon);
        assertEquals((4096/2.0)*5/Math.PI, EncoderUtil.fromVelocity(5, 4096, 1, 1, 0.5), Util.kEpsilon);
        assertEquals((1024/2.0), EncoderUtil.fromVelocity(Math.PI, 1024, 1, 1, 0.5), Util.kEpsilon);
        assertEquals((2048/2.0), EncoderUtil.fromVelocity(Math.PI, 1024, 2, 1, 0.5), Util.kEpsilon);
        assertEquals((1024/2.0), EncoderUtil.fromVelocity(Math.PI, 1024, 2, 2, 0.5), Util.kEpsilon);
        assertEquals((1024/2.0)*5/Math.PI, EncoderUtil.fromVelocity(5, 1024, 1, 1, 0.5), Util.kEpsilon);

        assertEquals((4096/10.0), EncoderUtil.fromVelocity(Math.PI, 4096, 1, 1, 0.1), Util.kEpsilon);
        assertEquals((8192/10.0), EncoderUtil.fromVelocity(Math.PI, 4096, 2, 1, 0.1), Util.kEpsilon);
        assertEquals((4096/10.0), EncoderUtil.fromVelocity(Math.PI, 4096, 2, 2, 0.1), Util.kEpsilon);
        assertEquals((4096/10.0)*5/Math.PI, EncoderUtil.fromVelocity(5, 4096, 1, 1, 0.1), Util.kEpsilon);
        assertEquals((1024/10.0), EncoderUtil.fromVelocity(Math.PI, 1024, 1, 1, 0.1), Util.kEpsilon);
        assertEquals((2048/10.0), EncoderUtil.fromVelocity(Math.PI, 1024, 2, 1, 0.1), Util.kEpsilon);
        assertEquals((1024/10.0), EncoderUtil.fromVelocity(Math.PI, 1024, 2, 2, 0.1), Util.kEpsilon);
        assertEquals((1024/10.0)*5/Math.PI, EncoderUtil.fromVelocity(5, 1024, 1, 1, 0.1), Util.kEpsilon);
    }

    
    @Test
    public void toRPMTest() {
        // Test Converting to RPM from Encoder Unit Velocity
        assertEquals(409.6, EncoderUtil.fromRPM(60, 4096, 1), Util.kEpsilon);
        assertEquals(4096, EncoderUtil.fromRPM(600, 4096, 1), Util.kEpsilon);
        assertEquals(819.2, EncoderUtil.fromRPM(60, 4096, 2), Util.kEpsilon);
        assertEquals(4096*83.2/600, EncoderUtil.fromRPM(83.2, 4096, 1), Util.kEpsilon);
        assertEquals(102.4, EncoderUtil.fromRPM(60, 1024, 1), Util.kEpsilon);
        assertEquals(1024, EncoderUtil.fromRPM(600, 1024, 1), Util.kEpsilon);
        assertEquals(204.8, EncoderUtil.fromRPM(60, 1024, 2), Util.kEpsilon);
        assertEquals(1024*83.2/600, EncoderUtil.fromRPM(83.2, 1024, 1), Util.kEpsilon);

        assertEquals(4096/2.0, EncoderUtil.fromRPM(60, 4096, 1, 0.5), Util.kEpsilon);
        assertEquals(40960/2.0, EncoderUtil.fromRPM(600, 4096, 1, 0.5), Util.kEpsilon);
        assertEquals(8192/2.0, EncoderUtil.fromRPM(60, 4096, 2, 0.5), Util.kEpsilon);
        assertEquals((4096*83.2/60)/2.0, EncoderUtil.fromRPM(83.2, 4096, 1, 0.5), Util.kEpsilon);
        assertEquals(1024/2.0, EncoderUtil.fromRPM(60, 1024, 1, 0.5), Util.kEpsilon);
        assertEquals(10240/2.0, EncoderUtil.fromRPM(600, 1024, 1, 0.5), Util.kEpsilon);
        assertEquals(2048/2.0, EncoderUtil.fromRPM(60, 1024, 2, 0.5), Util.kEpsilon);
        assertEquals((1024*83.2/60)/2.0, EncoderUtil.fromRPM(83.2, 1024, 1, 0.5), Util.kEpsilon);

        assertEquals(409.6, EncoderUtil.fromRPM(60, 4096, 1, 0.1), Util.kEpsilon);
        assertEquals(4096, EncoderUtil.fromRPM(600, 4096, 1, 0.1), Util.kEpsilon);
        assertEquals(819.2, EncoderUtil.fromRPM(60, 4096, 2, 0.1), Util.kEpsilon);
        assertEquals(4096*83.2/600, EncoderUtil.fromRPM(83.2, 4096, 1, 0.1), Util.kEpsilon);
        assertEquals(102.4, EncoderUtil.fromRPM(60, 1024, 1, 0.1), Util.kEpsilon);
        assertEquals(1024, EncoderUtil.fromRPM(600, 1024, 1, 0.1), Util.kEpsilon);
        assertEquals(204.8, EncoderUtil.fromRPM(60, 1024, 2, 0.1), Util.kEpsilon);
        assertEquals(1024*83.2/600, EncoderUtil.fromRPM(83.2, 1024, 1, 0.1), Util.kEpsilon);
    }

    @Test
    public void fromRPMTest() {
        // Test Converting from RPM to Encoder Unit Velocity
        assertEquals(60, EncoderUtil.toRPM(409.6, 4096, 1), Util.kEpsilon);
        assertEquals(600, EncoderUtil.toRPM(4096, 4096, 1), Util.kEpsilon);
        assertEquals(60, EncoderUtil.toRPM(819.2, 4096, 2), Util.kEpsilon);
        assertEquals(600*8371.0/4096.0, EncoderUtil.toRPM(8371, 4096, 1), Util.kEpsilon);
        assertEquals(60, EncoderUtil.toRPM(102.4, 1024, 1), Util.kEpsilon);
        assertEquals(600, EncoderUtil.toRPM(1024, 1024, 1), Util.kEpsilon);
        assertEquals(60, EncoderUtil.toRPM(204.8, 1024, 2), Util.kEpsilon);
        assertEquals(600*8371.0/1024.0, EncoderUtil.toRPM(8371, 1024, 1), Util.kEpsilon);

        assertEquals(12, EncoderUtil.toRPM(409.6, 4096, 1, 0.5), Util.kEpsilon);
        assertEquals(120, EncoderUtil.toRPM(4096, 4096, 1, 0.5), Util.kEpsilon);
        assertEquals(12, EncoderUtil.toRPM(819.2, 4096, 2, 0.5), Util.kEpsilon);
        assertEquals(120*8371.0/4096.0, EncoderUtil.toRPM(8371, 4096, 1, 0.5), Util.kEpsilon);
        assertEquals(12, EncoderUtil.toRPM(102.4, 1024, 1, 0.5), Util.kEpsilon);
        assertEquals(120, EncoderUtil.toRPM(1024, 1024, 1, 0.5), Util.kEpsilon);
        assertEquals(12, EncoderUtil.toRPM(204.8, 1024, 2, 0.5), Util.kEpsilon);
        assertEquals(120*8371.0/1024.0, EncoderUtil.toRPM(8371, 1024, 1, 0.5), Util.kEpsilon);

        assertEquals(60, EncoderUtil.toRPM(409.6, 4096, 1, 0.1), Util.kEpsilon);
        assertEquals(600, EncoderUtil.toRPM(4096, 4096, 1, 0.1), Util.kEpsilon);
        assertEquals(60, EncoderUtil.toRPM(819.2, 4096, 2, 0.1), Util.kEpsilon);
        assertEquals(600*8371.0/4096.0, EncoderUtil.toRPM(8371, 4096, 1, 0.1), Util.kEpsilon);
        assertEquals(60, EncoderUtil.toRPM(102.4, 1024, 1, 0.1), Util.kEpsilon);
        assertEquals(600, EncoderUtil.toRPM(1024, 1024, 1, 0.1), Util.kEpsilon);
        assertEquals(60, EncoderUtil.toRPM(204.8, 1024, 2, 0.1), Util.kEpsilon);
        assertEquals(600*8371.0/1024.0, EncoderUtil.toRPM(8371, 1024, 1, 0.1), Util.kEpsilon);
    }

}