package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.lib.drivers.TalonSRX;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Flywheel extends SubsystemBase {

    // Hardware
    private final WPI_TalonSRX mMaster;
    private final WPI_TalonSRX mFollower;

    // Hardware states
    private boolean mIsBrakeMode;

    // Logging
    private final Logger mLogger = LoggerFactory.getLogger( Flywheel.class );


    public void SetBrakeMode ( boolean wantsBrakeMode ) {
        if (wantsBrakeMode && !mIsBrakeMode) {
            mIsBrakeMode = wantsBrakeMode;
            mMaster.setNeutralMode( NeutralMode.Brake );
            mFollower.setNeutralMode( NeutralMode.Brake );
            mLogger.info( "Neutral mode set to: [Brake]" );

        } else if (!wantsBrakeMode && mIsBrakeMode) {
            mIsBrakeMode = wantsBrakeMode;
            mMaster.setNeutralMode( NeutralMode.Coast );
            mFollower.setNeutralMode( NeutralMode.Coast );
            mLogger.info( "Neutral mode set to: [Coast]" );
        }
    }

    public boolean IsBrakeMode () {
        return mIsBrakeMode;
    }

    public void OutputSmartDashboard () {
        if ( IsBrakeMode() ) {
            SmartDashboard.putString( "Neutral Mode", "Brake" );
        } else {
            SmartDashboard.putString( "Neutral Mode", "Coast" );
        }
    }

    public Flywheel ( WPI_TalonSRX master, WPI_TalonSRX follower ) {
  
        // Set the hardware
        mMaster = master;
        mFollower = follower;

        // Set the hardware states
        mIsBrakeMode = false;
        SetBrakeMode( true );
    }

    public static Flywheel create () {
        // Talon's and Victor's go through a custom wrapper for creation
        WPI_TalonSRX master = TalonSRX.createTalonSRXWithEncoder( new WPI_TalonSRX( Constants.FLYWHEEL_MASTER_ID) );
        WPI_TalonSRX follower = TalonSRX.createTalonSRX( new WPI_TalonSRX( Constants.FLYWHEEL_FOLLOWER_ID), master );
        return new Flywheel( master, follower );
    }

    @Override
    public void periodic () {

    }

}
