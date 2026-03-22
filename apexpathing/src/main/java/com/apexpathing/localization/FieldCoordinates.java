package com.apexpathing.localization;

import com.apexpathing.geometry.Pose2d;

/**
 * FTC field coordinate constants and utilities.
 * @Author Sohum Arora 22985
 */
public class FieldCoordinates {
    public static final double FIELD_SIZE = 144.0;
    public static final double HALF_FIELD = FIELD_SIZE / 2;

    public static final double MAX_X = HALF_FIELD;
    public static final double MIN_X = -HALF_FIELD;
    public static final double MAX_Y = HALF_FIELD;
    public static final double MIN_Y = -HALF_FIELD;

    public static Pose2d clamp(Pose2d pose) {
        return new Pose2d(
            Math.max(MIN_X, Math.min(MAX_X, pose.x)),
            Math.max(MIN_Y, Math.min(MAX_Y, pose.y)),
            pose.heading
        );
    }
}
