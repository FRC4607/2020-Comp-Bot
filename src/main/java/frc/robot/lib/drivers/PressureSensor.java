package frc.robot.lib.drivers;
import edu.wpi.first.wpilibj.AnalogInput;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class PressureSensor {
    private static final Logger mLogger = LoggerFactory.getLogger( PressureSensor.class );
    private final AnalogInput mAnalogInput;
    private final double mVoltsAtZeroPressure;
    private final double mPressurePerVolt;

    // https://www.automationdirect.com/adc/shopping/catalog/process_control_-a-_measurement/pressure_sensors/digital_pressure_switches_-z-_transmitters/qpsh-an-42
    public double GetPressureInPSI () {
        return ( mAnalogInput.getAverageVoltage() - mVoltsAtZeroPressure ) * mPressurePerVolt; 
        //return mAnalogInput.getAverageVoltage();  // Used for calibration
    }

    // The constructor expects the user to provide the constants to model volts-to-PSI model
    public PressureSensor ( int channel, double voltsAtZeroPressure, double pressurePerVolt ) {
        mAnalogInput = new AnalogInput( channel );
        mVoltsAtZeroPressure = voltsAtZeroPressure;
        mPressurePerVolt = pressurePerVolt;
        mAnalogInput.setOversampleBits( 6 );
        mAnalogInput.setAverageBits( 6 );
        mLogger.info( "Created pressure sensor [{}]", channel );
    }

}
