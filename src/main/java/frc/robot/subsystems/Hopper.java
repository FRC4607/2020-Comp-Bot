package frc.robot.subsystems;

import frc.robot.Constants.HOPPER;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class Hopper extends SubsystemBase {

    // Hardware
    private final CANSparkMax mHopperMotor;

    // Hardware states
    private boolean mIsBrakeMode;

    // Logging
    private final Logger mLogger = LoggerFactory.getLogger( Hopper.class );


    public void SetBrakeMode ( boolean wantsBrakeMode ) {
        if ( wantsBrakeMode && !mIsBrakeMode ) {
            mIsBrakeMode = wantsBrakeMode;
            mHopperMotor.setIdleMode( IdleMode.kBrake );
            mLogger.info( "Neutral mode set to: [Brake]" );

        } else if (!wantsBrakeMode && mIsBrakeMode) {
            mIsBrakeMode = wantsBrakeMode;
            mHopperMotor.setIdleMode( IdleMode.kCoast );
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

    public Hopper ( CANSparkMax hopperMotor ) {
  
        // Set the hardware
        mHopperMotor = hopperMotor;

        // Set the hardware states
        mIsBrakeMode = false;
        SetBrakeMode( true );
    }

    public static Hopper create () {
        // Talon's and Victor's go through a custom wrapper for creation
        CANSparkMax hopperMotor = new CANSparkMax( HOPPER.HOPPER_MOTOR_ID, MotorType.kBrushless );
        return new Hopper( hopperMotor );
    }

    @Override
    public void periodic () {

    }

}

