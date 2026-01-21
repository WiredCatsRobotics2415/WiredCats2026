package frc.utils;

import frc.robot.Robot;

public class Util {
    public static Object getIOImplementation(Class<?> realClass, Class<?> simClass, Object defaultImpl) {
        if (Robot.isReal()) {
            try {
                return realClass.getDeclaredConstructor().newInstance();
            } catch (Exception e) {
                e.printStackTrace();
            }
        } else {
            try {
                return simClass.getDeclaredConstructor().newInstance();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        return defaultImpl;
    }
}
