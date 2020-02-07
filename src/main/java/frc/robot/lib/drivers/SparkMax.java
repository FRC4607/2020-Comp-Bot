package frc.robot.lib.drivers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANError;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
* This class is used as a wrapper around the CANSparkMax object in order to initialize the motor controller to known
* configuration.  Also, any sticky errors present will be logged and cleared.
*  
* @see {@link http://www.revrobotics.com/content/sw/max/sw-docs/java/index.html}
*/    

public class SparkMax {

    private static final Logger mLogger = LoggerFactory.getLogger( SparkMax.class );

    /**
    * First, any motor controller sticky faults will be logged and cleared.  Next, the default configuration for the
    * motor controller is reset to factory defaults.  Finally, voltage compensation will be enabled and set for 12V.
    *  
    * @see {@link frc.robot.lib.drivers.SparkMax}
    */        
    private static void SetDefaultConfig ( CANSparkMax sparkMax ) {

        final long faults = sparkMax.getStickyFaults();
        if ( faults != 0 ) {
            mLogger.warn( "Clearing SparkMax [{}] sticky faults: [{}]", sparkMax.getDeviceId(), faults );
            final CANError clearStickyFaults = sparkMax.clearFaults();
            if ( clearStickyFaults != CANError.kOk ) {
                mLogger.error( "Could not clear sticky faults due to EC: [{}]", clearStickyFaults );
            }  
        }
        
        final CANError configFactoryDefault = sparkMax.restoreFactoryDefaults();
        if ( configFactoryDefault != CANError.kOk ) {
            mLogger.error( "Could not factory reset SparkMax [{}] due to EC: [{}]", sparkMax.getDeviceId(), configFactoryDefault.toString() );
        }  

        final CANError configVoltageCompSaturation = sparkMax.enableVoltageCompensation( 12.0 );
        if ( configVoltageCompSaturation != CANError.kOk ) {
            mLogger.error( "Could not set SparkMax [{}] voltage compensation due to EC: [{}]", sparkMax.getDeviceId(), configVoltageCompSaturation.toString() );
        }  

    }

    /**
    * Create a master CANSparkMax motor controller.
    *  
    * @see {@link frc.robot.lib.drivers.SparkMax}
    */  
    public static CANSparkMax CreateSparkMax ( CANSparkMax sparkMax ) {
        SetDefaultConfig( sparkMax );
        return sparkMax;
    }

    /**
    * Create a follower CANSparkMax motor controller.
    *  
    * @see {@link frc.robot.lib.drivers.SparkMax}
    */  
    public static CANSparkMax CreateSparkMax ( CANSparkMax sparkMax, CANSparkMax master ) {
        SetDefaultConfig( sparkMax );
        sparkMax.follow( master );
        return sparkMax;
    }

}