package frc.robot.subsystems;

import frc.robot.Constants.INDEXER;
import frc.robot.Constants.CURRENT_LIMIT;
import frc.robot.lib.drivers.Photoeye;
import frc.robot.lib.drivers.SparkMax;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class Indexer extends SubsystemBase {

    // Hardware
    private CANSparkMax mMaster;
    // private Photoeye mIndexerPhotoeye = new Photoeye( INDEXER.PHOTOEYE_DIO_CHANNEL );

    // Logging
    private final Logger mLogger = LoggerFactory.getLogger( Indexer.class );

    // spin indexer sending power cells into transfer
    public void Spin () {
        mMaster.set( INDEXER.SPEED );
        mMaster.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
    }

    // spin hopper backward away from indexer
    public void SpinBack () {
        mMaster.set( -INDEXER.SPEED );
        mMaster.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
    }

    // stop spinning
    public void Stop () {
        mMaster.set( 0.0 );
        mMaster.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
    }

    public Indexer ( CANSparkMax master ) {
        mMaster = master;
        // Current limiting
        mMaster.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
    }

    public static Indexer create () {
        CANSparkMax master =  SparkMax.CreateSparkMax( new CANSparkMax( INDEXER.MASTER_ID, MotorType.kBrushless ) );
        return new Indexer( master );
    }

    @Override
    public void periodic () {

    }

}

