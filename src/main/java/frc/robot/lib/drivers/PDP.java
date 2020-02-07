package frc.robot.lib.drivers;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class PDP {

    private static final Logger mLogger = LoggerFactory.getLogger( PDP.class );

    // Create PDP
    public static PowerDistributionPanel createPDP ( PowerDistributionPanel pdp, int deviceID ) {
        
        pdp.clearStickyFaults();
        mLogger.info( "Created power distribution panel [{}]", deviceID );

        return pdp;
    }    

}