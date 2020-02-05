package frc.robot.lib.drivers;
import edu.wpi.first.wpilibj.AnalogInput;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class Photoeye {
    private static final Logger mLogger = LoggerFactory.getLogger( Photoeye.class );
    private final double mThreshold = 1.0;
    private final AnalogInput mAnalogInput;

    public boolean IsPhotoeyeClosed () {
        return mAnalogInput.getAverageVoltage() > mThreshold ?  false : true;  // PNP
    }

    public Photoeye(int channel) {
        mAnalogInput = new AnalogInput( channel );
        mAnalogInput.setOversampleBits( 6 );
        mAnalogInput.setAverageBits( 6 );
        mLogger.info( "Created photoeye [{}]", channel );
    }

}
