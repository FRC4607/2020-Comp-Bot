package frc.robot.subsystems;

import frc.robot.Constants.TRANSFER_WHEEL;
import frc.robot.Constants.CURRENT_LIMIT;
import frc.robot.lib.drivers.Photoeye;
import frc.robot.lib.drivers.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StickyFaults;

public class TransferWheel extends SubsystemBase {

    // Hardware
    private CANSparkMax mMaster;
    // private boolean mIsSpin;
    //private Photoeye mIndexerPhotoeye = new Photoeye( TRANSFER_WHEEL.PHOTOEYE_DIO_CHANNEL );

    // Logging
    private final Logger mLogger = LoggerFactory.getLogger( TransferWheel.class );

    public void Spin () {
        // if ( wantsSpin != mIsSpin ) {
            // mIsSpin = wantsSpin; 
        mMaster.set( TRANSFER_WHEEL.SPEED );
        mMaster.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
        // }
    }

    // public boolean IsSpin () {
    //     return mIsSpin;
    // }

    public void Stop () {
        mMaster.set( 0.0 );
        mMaster.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
    }


    public TransferWheel ( CANSparkMax master ) {
        mMaster = master;
        // Current limiting
        mMaster.setSmartCurrentLimit( CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, CURRENT_LIMIT.SPARK_RPM_LIMIT );
        // Set the hardware states
        // mIsSpin = false; 
        // Spin( false ); 
    
    }
    
    public static TransferWheel create () {
        CANSparkMax master =  SparkMax.CreateSparkMax( new CANSparkMax( TRANSFER_WHEEL.MASTER_ID, MotorType.kBrushless ) );
        return new TransferWheel( master );
    }
}
