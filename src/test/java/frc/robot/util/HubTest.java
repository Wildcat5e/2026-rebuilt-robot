package frc.robot.util;

import org.junit.jupiter.api.Test;

import static frc.test.Functions.pose2d;
import static org.junit.jupiter.api.Assertions.assertEquals;

class HubTest {

    @Test void distanceTo() {
        assertEquals(6.13, Hub.BLUE.distanceTo(pose2d(0.0, 0.0, 0.0)), .01);
        assertEquals(4.15, Hub.BLUE.distanceTo(pose2d(1.0, 2.0, Math.PI)), .01);
        assertEquals(3.32, Hub.BLUE.distanceTo(pose2d(2.0, 2.0, Math.PI / 4)), .01);
        assertEquals(1.20, Hub.BLUE.distanceTo(pose2d(4.0, 3.0, Math.PI / 2)), .01);
    }


    @Test void angleTo() {
        assertEquals(0.72, Hub.BLUE.angleTo(pose2d(0.0, 0.0, 0.0)), .01);
        assertEquals(0.51, Hub.BLUE.angleTo(pose2d(1.0, 2.0, Math.PI)), .01);
        assertEquals(0.66, Hub.BLUE.angleTo(pose2d(2.0, 2.0, Math.PI / 4)), .01);
        assertEquals(1.03, Hub.BLUE.angleTo(pose2d(4.0, 3.0, Math.PI / 2)), .01);
    }

}