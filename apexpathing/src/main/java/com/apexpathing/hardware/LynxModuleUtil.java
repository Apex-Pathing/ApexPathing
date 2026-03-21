package com.apexpathing.hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

/**
 * Utility for Lynx Module bulk read optimization.
 */
public class LynxModuleUtil {
    public static void setBulkCachingModeManual(HardwareMap hardwareMap) {
        List<LynxModule> modules = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : modules) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public static void clearBulkCache(HardwareMap hardwareMap) {
        List<LynxModule> modules = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : modules) {
            module.clearBulkCache();
        }
    }
}
