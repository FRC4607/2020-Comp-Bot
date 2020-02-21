package frc.robot.subsystems;

import frc.robot.Constants.CLIMBER;
import frc.robot.Constants.CURRENT_LIMIT;
import frc.robot.lib.drivers.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    // Hardware
    private final CANSparkMax mMaster;

    public void OpenLoop() {
        mMaster.set( CLIMBER.SPEED );
        mMaster.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
    }

    // stop for deadband
    public void Stop() {
        mMaster.set( 0.0 );
        mMaster.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
    }

    public Climber( CANSparkMax master /*  Follower, CANPIDController PidController*/ ) {
 
        mMaster = master;
    
        // current limiting for sparks by zero rpm, max rpm, and inbetween rpm current limits
        mMaster.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
    }

    public static Climber create () {
        CANSparkMax master =  SparkMax.CreateSparkMax( new CANSparkMax( CLIMBER.MASTER_ID, MotorType.kBrushless ) );
        return new Climber( master /* Follower, PidController*/ ); 
    } 

    @Override
    public void periodic () {
    }

}
