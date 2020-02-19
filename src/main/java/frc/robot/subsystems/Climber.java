package frc.robot.subsystems;

import frc.robot.Constants.GLOBAL;
import frc.robot.Constants.CLIMBER;
import frc.robot.Constants.CURRENT_LIMIT;
import frc.robot.lib.drivers.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends SubsystemBase {

    // Hardware
    private final CANSparkMax mMaster;
    private final CANSparkMax mFollower;
    private CANEncoder mAlternateEncoder;
    private CANPIDController mPIDController;

    // Logging
    private final Logger mLogger = LoggerFactory.getLogger( Climber.class );

    public Climber ( CANSparkMax master, CANSparkMax follower, CANEncoder alternateEncoder, CANPIDController pidController ) {
        
        // Set the hardware
        mMaster = master;
        mFollower = follower;
        mAlternateEncoder = alternateEncoder;
        mPIDController = pidController;

        // Current limiting
        mMaster.setSmartCurrentLimit( 6, 6, CURRENT_LIMIT.RPM_LIMIT );
        mFollower.setSmartCurrentLimit( 6, 6, CURRENT_LIMIT.RPM_LIMIT );

    }

    public static Drivetrain create () {
        CANSparkMax master =  SparkMax.CreateSparkMax( new CANSparkMax( CLIMBER.MASTER_ID, MotorType.kBrushless ) );
        CANSparkMax follower =  SparkMax.CreateSparkMax( new CANSparkMax( CLIMBER.FOLLOWER_ID, MotorType.kBrushless ), master );
        CANEncoder alternateEncoder = master.getAlternateEncoder( AlternateEncoderType.kQuadrature, CLIMBER.SENSOR_COUNTS_PER_ROTATION );
        CANPIDController pidController = master.getPIDController();
       return new Climber( master, follower, alternateEncoder, pidController ); 
    }

    @Override
    public void periodic () {
    }

}

