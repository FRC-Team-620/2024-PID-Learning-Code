package frc.robot;

public class Constants {
    public static final double ARM_MOTOR_POSITION_CONVERSION_FACTOR_DEGREES = ( 1.0 / 6.0 ) * 360.0;
    public static final double SIMPLE_MOTOR_SPEED = 0.3;
    public static final double BANG_BANG_CONTROL_SPEED = 0.2;
    public static final int DEVICE_ID = 59;
    public static final int CONTROL_LOOP_ERROR_DEGREES = 5;

    // 0.002 is perfect for just proportional gain
    public static final double ARM_K_proportional = 0.002; // very stable
    // public static final double ARM_K_proportional = 0.0071; // unstable slow damping

    public static final double ARM_MAX_OUTPUT = 0.6;

}
