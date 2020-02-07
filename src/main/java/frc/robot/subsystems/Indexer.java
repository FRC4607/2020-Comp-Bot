package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.lib.drivers.Photoeye;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class Indexer extends SubsystemBase {

    // Hardware
    private final CANSparkMax mIndexerMotor;
    private final Photoeye mIndexerPhotoeye = new Photoeye( Constants.INDEXER_PHOTOEYE_ANALOG_CHANNEL );

    // Hardware states
    private boolean mIsBrakeMode;

    // Logging
    private final Logger mLogger = LoggerFactory.getLogger( Indexer.class );


    public void SetBrakeMode ( boolean wantsBrakeMode ) {
        if (wantsBrakeMode && !mIsBrakeMode) {
            mIsBrakeMode = wantsBrakeMode;
            mIndexerMotor.setIdleMode(IdleMode.kBrake);
      mLogger.info("Neutral mode set to: [Brake]");

    } else if (!wantsBrakeMode && mIsBrakeMode) {
      mIsBrakeMode = wantsBrakeMode;
      mIndexerMotor.setIdleMode(IdleMode.kCoast);
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

    public Indexer ( CANSparkMax indexerMotor ) {
  
        // Set the hardware
        mIndexerMotor = indexerMotor;

        // Set the hardware states
        mIsBrakeMode = false;
        SetBrakeMode( true );
    }

    public static Indexer create () {
        // Talon's and Victor's go through a custom wrapper for creation
        CANSparkMax indexerMotor = new CANSparkMax( Constants.INDEXER_MOTOR_ID, MotorType.kBrushless );
        return new Indexer( indexerMotor );
    }

    @Override
    public void periodic () {

    }

}

