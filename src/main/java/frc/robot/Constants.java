package frc.robot;

public final class Constants {

    // Pressure sensor
    public static final int PRESSURE_SENSOR_ANALOG_CHANNEL = 0;
    public static final double PRESSURE_SENSOR_VOLTS_AT_ZERO_PRESSURE = 1.19;               // Measure by reading analog input voltage @ 0-PSI 
    public static final double PRESSURE_SENSOR_PRESSURE_PER_VOLT = 115.0 / (3.62 - 1.19);   // Calculate with prior measurement and reading analog input voltage @ max operating PSI 

    // Photoeye
    public static final int PHOTOEYE_ANALOG_CHANNEL = 1;

    // Drivetrain device ID's and ports
    public static final int DRIVETRAIN_LEFT_MASTER_ID = 15;
    public static final int DRIVETRAIN_LEFT_FOLLOWER_ID = 14;
    public static final int DRIVETRAIN_RIGHT_MASTER_ID = 0;
    public static final int DRIVETRAIN_RIGHT_FOLLOWER_ID = 1;
    public static final int DRIVETRAIN_LOW_GEAR_SOLENOID_ID = 0;
    public static final int DRIVETRAIN_HIGH_GEAR_SOLENOID_ID = 1;

    // Flywheel device ID's
    public static final int FLYWHEEL_MASTER_ID = 10000;
    public static final int FLYWHEEL_FOLLOWER_ID = 10000;

    // Hood devices ID's
    public static final int HOOD_MOTOR_ID = 10;
    public static final int SMART_MOTION_ID = 11;

    // Hopper devices ID's
    public static final int HOPPER_MOTOR_ID = 10;

    // Indexer devices ID's
    public static final int INDEXER_PHOTOEYE_ANALOG_CHANNEL = 0;
    public static final int INDEXER_MOTOR_ID = 10;

    // Intake device ID's
    public static final int INTAKE_MOTOR_ID = 10;

    // Tansfer device ID's
    public static final int TRANSFER_PHOTOEYE_ANALOG_CHANNEL = 1;
    public static final int TRANSFER_MOTOR_ID = 10;


    // Controllers and Joysticks
    public static final int DRIVER_XBOX = 0;
    public static final int OPERATOR_XBOX = 1;

    // CAN bus
    public static final int CAN_TIMEOUT_MS = 10;
    public static final int CAN_LONG_TIMEOUT_MS = 100;

    // MISC. Hardware Device ID's
    public static final int PCM_ID = 0;
    public static final int PDP_ID = 0;

}
