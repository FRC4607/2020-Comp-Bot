package frc.robot.subsystems;

import frc.robot.Constants.HOPPER;
import frc.robot.Constants.CURRENT_LIMIT;
import frc.robot.lib.drivers.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class Hopper extends SubsystemBase {

    // Hardware
    private CANSparkMax mMaster; 

    // Logging
    private final Logger mLogger = LoggerFactory.getLogger( Hopper.class );

    // spin hopper sending power cells into indexer
    public void Spin () {
        mMaster.set( HOPPER.SPEED );
        mMaster.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
    }

    // spin hopper backward away from indexer
    public void SpinBack () {
        mMaster.set( -HOPPER.SPEED );
        mMaster.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
    }

    public void Stop () {
        mMaster.set( 0.0 );
        mMaster.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
    }

    public Hopper ( CANSparkMax master ) {
        mMaster = master;
        // Current limiting
        mMaster.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
    }

    public static Hopper create () {
        CANSparkMax master =  SparkMax.CreateSparkMax( new CANSparkMax( HOPPER.MASTER_ID, MotorType.kBrushless ) );
        return new Hopper( master );
    }

    @Override
    public void periodic () {

    }

}

