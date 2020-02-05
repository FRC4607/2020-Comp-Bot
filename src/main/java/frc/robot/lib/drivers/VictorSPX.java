package frc.robot.lib.drivers;

import frc.robot.Constants;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class VictorSPX {

    private static final Logger mLogger = LoggerFactory.getLogger( VictorSPX.class );

    // Set the default configuration for the Victor (this includes first resetting to factory defaults)
    private static void setDefaultConfig( WPI_VictorSPX victor ) {
        StickyFaults faults = new StickyFaults();

        victor.getStickyFaults( faults );
        if ( faults.hasAnyFault() ) {
            mLogger.warn( "Clearing VictorSPX [{}] sticky faults: [{}]", victor.getDeviceID(), faults.toString() );
            final ErrorCode clearStickyFaults = victor.clearStickyFaults( Constants.CAN_LONG_TIMEOUT_MS );
            if ( clearStickyFaults != ErrorCode.OK ) {
                mLogger.error( "Could not clear sticky faults due to EC: [{}]", clearStickyFaults );
            }  
        }
    
        final ErrorCode configFactoryDefault = victor.configFactoryDefault();
        if ( configFactoryDefault != ErrorCode.OK ) {
            mLogger.error( "Could not factory reset VictorSPX [{}] due to EC: [{}]", victor.getDeviceID(), configFactoryDefault );
        }  

        final ErrorCode configVoltageCompSaturation = victor.configVoltageCompSaturation( 12.0, Constants.CAN_LONG_TIMEOUT_MS );
        if ( configVoltageCompSaturation != ErrorCode.OK ) {
            mLogger.error( "Could not set VictorSPX [{}] voltage compensation due to EC: [{}]", victor.getDeviceID(), configVoltageCompSaturation );
        }  

        victor.enableVoltageCompensation( true );
    }

    // Create master
    public static WPI_VictorSPX createVictorSPX ( WPI_VictorSPX victor ) {
        setDefaultConfig( victor );
        victor.set( ControlMode.PercentOutput, 0.0 );
        mLogger.info(" Created leader VictorSPX [{}]", victor.getDeviceID() );
        return victor;
    }
  
    // Create follower
    public static WPI_VictorSPX createVictorSPX ( WPI_VictorSPX victor, WPI_TalonSRX master ) {
        setDefaultConfig( victor );
        victor.follow( master );
        mLogger.info( "Created follower VictorSPX [{}]", victor.getDeviceID() );
        return victor;
    }

}
