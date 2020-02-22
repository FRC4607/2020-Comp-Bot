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
        } else if ( !wantsUp && mIsUp ) {
            mIsUp = wantsUp; 
            mShifter.set( DoubleSolenoid.Value.kReverse );
        }
    }
    public boolean IsUp() {
        return mIsUp;
    }

    // open loop drive
    public void setOpenLoop (double xIntake) {
        mMaster.set( xIntake );
        mMaster.configContinuousCurrentLimit( CURRENT_LIMIT.TALON_AMPS_LIMIT );
        mMaster.configPeakCurrentLimit( CURRENT_LIMIT.TALON_AMPS_LIMIT );
        mMaster.configPeakCurrentDuration( GLOBAL.TALON_CURRENT_LIMIT_TIMEOUT_MS );
        mMaster.enableCurrentLimit( true );
    }
    // stop for deadband
    public void Stop() {
        mMaster.set( 0.0 );
        mMaster.configContinuousCurrentLimit( CURRENT_LIMIT.TALON_AMPS_LIMIT );
        mMaster.configPeakCurrentLimit( CURRENT_LIMIT.TALON_AMPS_LIMIT );
        mMaster.configPeakCurrentDuration( GLOBAL.TALON_CURRENT_LIMIT_TIMEOUT_MS );
        mMaster.enableCurrentLimit( true );
    }

    public Intake ( WPI_TalonSRX master, DoubleSolenoid shifter ) {
        mMaster = master;
        mShifter = shifter;
        // Current limiting
        mMaster.configContinuousCurrentLimit( CURRENT_LIMIT.TALON_AMPS_LIMIT );
        mMaster.configPeakCurrentLimit( CURRENT_LIMIT.TALON_AMPS_LIMIT );
        mMaster.configPeakCurrentDuration( GLOBAL.TALON_CURRENT_LIMIT_TIMEOUT_MS );
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