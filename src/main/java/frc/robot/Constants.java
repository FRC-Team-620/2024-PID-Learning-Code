package frc.robot;

public class Constants {
    public static final double SIMPLE_MOTOR_SPEED = 0.3;
    public static final int DEVICE_ID = 59;
    public static final int CONTROL_LOOP_ERROR = 1;
    // 0.016 becomes unstable
    // 0.002 is perfect for just proportional
    public static final double ARM_K_proportional = 0.002;
    public static final double ARM_MAX_OUTPUT = 0.6;

}
