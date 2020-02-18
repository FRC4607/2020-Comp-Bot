package frc.robot.subsystems;

import frc.robot.Constants.INTAKE;
import frc.robot.Constants.GLOBAL;
import frc.robot.Constants.CURRENT_LIMIT;
import frc.robot.lib.drivers.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class Intake extends SubsystemBase {
    // Get some Current Limiting so the motor dosen't burn out
    
    // Hardware
    private WPI_TalonSRX mMaster;
    private final DoubleSolenoid mShifter;

    // Hardware states
    private boolean mIsUp;

    // Logging
    private final Logger mLogger = LoggerFactory.getLogger( Intake.class );

    // shift intake up and down
    public void SetUp ( boolean wantsUp ) {
        if ( wantsUp && !mIsUp ) {
            mIsUp = wantsUp;
            mShifter.set( DoubleSolenoid.Value.kForward );
            mLogger.info( "Gear set to: [High]" );
        } else if ( !wantsUp && mIsUp ) {
            mIsUp = wantsUp; 
            mShifter.set( DoubleSolenoid.Value.kReverse );
            mLogger.info( "Gear set to: [Low]" );
        }
    }
    public boolean IsUp() {
        return mIsUp;
    }

    // open loop drive
    public void setOpenLoop (double xIntake) {
        mMaster.set( xIntake );
    }

    // stop for deadband
    public void Stop() {
        mMaster.set( 0.0 );
    }

    public Intake ( WPI_TalonSRX master, DoubleSolenoid shifter ) {
        mMaster = master;
        mShifter = shifter;
    // Current limiting
        mMaster.configContinuousCurrentLimit( 6, CURRENT_LIMIT.LONG_CAN_TIMEOUT_MS );
        mMaster.configPeakCurrentLimit( 6, CURRENT_LIMIT.LONG_CAN_TIMEOUT_MS );
        mMaster.configPeakCurrentDuration( 200, CURRENT_LIMIT.LONG_CAN_TIMEOUT_MS );
        mMaster.enableCurrentLimit( true );
    }

    public static Intake create () {
        WPI_TalonSRX master = TalonSRX.createTalonSRXWithEncoder( new WPI_TalonSRX( INTAKE.MASTER_ID) );
        DoubleSolenoid shifter = new DoubleSolenoid( GLOBAL.PCM_ID, INTAKE.UP_SOLENOID_ID, INTAKE.DOWN_SOLENOID_ID );
        return new Intake( master, shifter );
    }

    @Override
    public void periodic () {

    }

}