package frc.robot;

public final class Constants {

    public static final class GLOBAL {
        public static final int CAN_TIMEOUT_MS = 10;
        public static final int CAN_LONG_TIMEOUT_MS = 100;
        public static final int PCM_ID = 0;
        public static final int PDP_ID = 0;
    }

    /**
    * These are the constants which are used to map the hardware and define the working bahavior of the the flywheel
    * subsystem.
    * @see {@link frc.robot.subsystems.Flywheel}
    * @see {@link https://phoenix-documentation.readthedocs.io/en/latest/ch16_ClosedLoop.html#}
    */        
    public static final class FLYWHEEL {
        public static final int MASTER_ID = 13;                      // The device ID of the master *THIS SHOULD MATCH THE PDP SLOT IT IS CONNECTED TO
        public static final int FOLLOWER_ID = 14;                    // The device ID of the follower *THIS SHOULD MATCH THE PDP SLOT IT IS CONNECTED TO
        public static final int SENSOR_UNITS_PER_ROTATION = 4096;       // Using a CTRE Mag Encoder
        public static final int PID_IDX = 0;                            // Velocity closed-loop slot index for gains
        public static final double PID_KP = 0.0;                        // Velocity closed-loop proportional gain
        public static final double PID_KI = 0.0;                        // Velocity closed-loop intgral gain
        public static final double PID_KD = 0.0;                        // Velocity closed-loop derivative gain
        public static final double PID_KF = 0.0;                        // Velocity closed-loop feed-forward
        public static final double OFFTRACK_ERROR_PERCENT = 0.01;       // This limit determines when the flywheel is considered on-target
        public static final double SEEK_TIMER_EXPIRED_S = 2.0;          // Allow 2 seconds to spin up to target velocity before retrying
        public static final int SEEK_RETRY_LIMIT = 3;                   // Allow 3 seek retries before falling back to open-loop 
        public static final double PERCENT_MOTOR_OUTPUT = 0.5;          // The open-loop motor percent ouput
        public static final double DEADBAND = 0.1; 
    }

    /**
    * These are the constants which are used to map the hardware and define the working bahavior of the the hood
    * subsystem.
    * @see {@link frc.robot.subsystems.Hood}
    */        
    public static final class HOOD {
        public static final int MASTER_ID = 5;                         // The device ID of the master *THIS SHOULD MATCH THE PDP SLOT IT IS CONNECTED TO
        public static final int SENSOR_COUNTS_PER_ROTATION = 8192;         // Using a REV Through Bore Encoder
        public static final int PID_IDX = 0;                               // Position closed-loop slot index for gains
        public static final double PID_KP = 0.0;                           // Position closed-loop proportional gain
        public static final double PID_KI = 0.0;                           // Position closed-loop intgral gain
        public static final double PID_KD = 0.0;                           // Position closed-loop derivative gain
        public static final double PID_KF = 0.0;                           // Position closed-loop feed-forward
        public static final int MOVING_AVERAGE_FILTER_TAPS = 5;            // The number of samples to average the position for zero'ing the sensor
        public static final double ZEROING_MOTOR_OUTPUT = 0.1;             // The open-loop motor output percentage for zero'ing the sensor
        public static final double ZEROING_TIMER_EXPIRED_S = 0.5;          // The time in seconds that the hood will search for the hard-stop
        public static final double ZEROING_DONE_THRESHOLD = 50.0 / 8192.0; // The threshold used to compare against the current postion - filtered position
        public static final int ZEROING_RETRY_LIMIT = 1;                   // Allow 3 zeroing retries before failing and living with a fixed hood
        public static final double MAX_VELOCITY = 1000.0;                  // Smart Motion max velocity
        public static final double MAX_ACCELERATION = 500.0;               // Smart Motion max acceleration
        public static final double DEADBAND = 0.1; 
    }

    /**
    * These are the constants which are used to map the hardware and define the working bahavior of the the turret
    * subsystem.
    * @see {@link frc.robot.subsystems.Turret}
    */        
    public static final class TURRET {
        public static final int MASTER_ID = 4;                             // The device ID of the master *THIS SHOULD MATCH THE PDP SLOT IT IS CONNECTED TO
        public static final int SENSOR_COUNTS_PER_ROTATION = 8192;         // Using a REV Through Bore Encoder
        public static final int PID_IDX = 0;                               // Position closed-loop slot index for gains
        public static final double PID_KP = 0.0;                           // Position closed-loop proportional gain
        public static final double PID_KI = 0.0;                           // Position closed-loop intgral gain
        public static final double PID_KD = 0.0;                           // Position closed-loop derivative gain
        public static final double PID_KF = 0.0;                           // Position closed-loop feed-forward
        public static final int MOVING_AVERAGE_FILTER_TAPS = 5;            // The number of samples to average the position for zero'ing the sensor
        public static final double ZEROING_MOTOR_OUTPUT = 0.1;             // The open-loop motor output percentage for zero'ing the sensor
        public static final double ZEROING_TIMER_EXPIRED_S = 0.5;          // The time in seconds that the hood will search for the hard-stop
        public static final double ZEROING_DONE_THRESHOLD = 50.0 / 8192.0; // The threshold used to compare against the current postion - filtered position
        public static final int ZEROING_RETRY_LIMIT = 1;                   // Allow 3 zeroing retries before failing and living with a fixed hood
        public static final double MAX_VELOCITY = 1000.0;                  // Smart Motion max velocity
        public static final double MAX_ACCELERATION = 500.0;               // Smart Motion max acceleration
        public static final double DEADBAND = 0.1; 
        public static final double TURNING_GAIN = 1.0;                     // SPEED OF TURN gain
        public static final double STOP_TURNING_DEG = 1.0; 
    }

    /**
    * These are the constants which are used to map the hardware and define the working bahavior of the the drivetrain
    * subsystem.
    * @see {@link frc.robot.subsystems.Drivetrain}
    */        
    public static final class DRIVETRAIN {
        public static final int LEFT_MASTER_ID = 16;
        public static final int LEFT_FOLLOWER_ID = 1;
        public static final int RIGHT_MASTER_ID = 2;
        public static final int RIGHT_FOLLOWER_ID = 3;
        public static final int SENSOR_COUNTS_PER_ROTATION = 8192;         // Using a REV Through Bore Encoder
        public static final int LOW_GEAR_SOLENOID_ID = 3;
        public static final int HIGH_GEAR_SOLENOID_ID = 6;
    }

    /**
    * These are the constants which are used to map the hardware and define the working bahavior of the the hopper
    * subsystem.
    * @see {@link frc.robot.subsystems.Hopper}
    */ 
    public static final class HOPPER {
        // is supposed to be 15 
        public static final int MASTER_ID = 15;
        public static final double SPEED = 0.5;
    }    

    /**
    * These are the constants which are used to map the hardware and define the working bahavior of the the indexer
    * subsystem.
    * @see {@link frc.robot.subsystems.Indexer}
    */        
    public static final class INDEXER {
        public static final int MASTER_ID = 6;
        public static final int PHOTOEYE_DIO_CHANNEL = 0;
        public static final double SPEED = -0.5;
    }    

    /**
    * These are the constants which are used to map the hardware and define the working bahavior of the the intake
    * subsystem.
    * @see {@link frc.robot.subsystems.Intake}
    */        
    public static final class INTAKE {
        public static final int MASTER_ID = 10; 
        public static final int UP_SOLENOID_ID = 2; 
        public static final int DOWN_SOLENOID_ID = 7; 
        public static final double DEADBAND = 0.1; 
    }    

    /**
    * These are the constants which are used to map the hardware and define the working bahavior of the the transfer
    * wheel subsystem.
    * @see {@link frc.robot.subsystems.TransferWheel}
    */        
    public static final class TRANSFER_WHEEL {
        // supposed to be 11
        public static final int MASTER_ID = 11;
        public static final int PHOTOEYE_DIO_CHANNEL = 1;
        public static final double SPEED = 1.0;
    }   

    // Controllers and Joysticks
    public static final int DRIVER_XBOX = 0;
    public static final int OPERATOR_XBOX = 1;

    // Pressure sensor
    public static final int PRESSURE_SENSOR_ANALOG_CHANNEL = 0;
    public static final double PRESSURE_SENSOR_VOLTS_AT_ZERO_PRESSURE = 1.19;               // Measure by reading analog input voltage @ 0-PSI 
    public static final double PRESSURE_SENSOR_PRESSURE_PER_VOLT = 115.0 / (3.62 - 1.19);   // Calculate with prior measurement and reading analog input voltage @ max operating PSI 

    // Photoeye
    public static final int PHOTOEYE_ANALOG_CHANNEL = 1; 

    // limelight
    public static final double SCALE_HORIZONTAL_TO_TARGET = 1.0 / 27.0;                // Limelight has 54deg FOV
    public static final double VISION_THREAD_TIME = 0.01;   

}

