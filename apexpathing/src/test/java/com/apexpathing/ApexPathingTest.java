package com.apexpathing;

import static org.junit.Assert.*;
import org.junit.Test;
import com.apexpathing.geometry.Vector2d;
import com.apexpathing.follower.QuinticHermiteSpline;

public class ApexPathingTest {
    @Test
    public void testVectorMath() {
        Vector2d v1 = new Vector2d(1, 2);
        Vector2d v2 = new Vector2d(3, 4);

        Vector2d v3 = v1.plus(v2);
        assertEquals(4, v3.x, 1e-9);
        assertEquals(6, v3.y, 1e-9);

        Vector2d v4 = v1.rotate(Math.PI / 2);
        assertEquals(-2, v4.x, 1e-9);
        assertEquals(1, v4.y, 1e-9);
    }

    @Test
    public void testQuinticHermiteSpline() {
        Vector2d p0 = new Vector2d(0, 0);
        Vector2d v0 = new Vector2d(1, 0);
        Vector2d a0 = new Vector2d(0, 0);

        Vector2d p1 = new Vector2d(1, 1);
        Vector2d v1 = new Vector2d(0, 1);
        Vector2d a1 = new Vector2d(0, 0);

        QuinticHermiteSpline spline = new QuinticHermiteSpline(p0, v0, a0, p1, v1, a1);

        Vector2d start = spline.getPoint(0);
        assertEquals(0, start.x, 1e-9);
        assertEquals(0, start.y, 1e-9);

        Vector2d end = spline.getPoint(1);
        assertEquals(1, end.x, 1e-9);
        assertEquals(1, end.y, 1e-9);

        Vector2d startVel = spline.getVelocity(0);
        assertEquals(1, startVel.x, 1e-9);
        assertEquals(0, startVel.y, 1e-9);

        Vector2d endVel = spline.getVelocity(1);
        assertEquals(0, endVel.x, 1e-9);
        assertEquals(1, endVel.y, 1e-9);
    }
}
