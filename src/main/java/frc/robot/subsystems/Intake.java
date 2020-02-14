package frc.robot.subsystems;

import frc.robot.Constants.INTAKE;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.lib.drivers.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class Intake extends SubsystemBase {

    // Hardware states
    private boolean mIsBrakeMode;
    private boolean mIsOuttake;

    // Hardware
    private WPI_TalonSRX mIntakeMotor;

    // Logging
    private final Logger mLogger = LoggerFactory.getLogger( Intake.class );

    // reverse direction of intaking / outtaking
    public void SetOuttake ( boolean wantsOuttake ) {
        if ( wantsOuttake != mIsOuttake ) {
            mIsOuttake = wantsOuttake;
            mIntakeMotor.setInverted( wantsOuttake );
            mLogger.info( "Reversed intake drive set to: [{}]", mIsOuttake );
        }
    }

    public boolean IsOuttake () {
        return mIsOuttake;
    }

    public void SetBrakeMode ( boolean wantsBrakeMode ) {
        if ( wantsBrakeMode && !mIsBrakeMode ) {
            mIsBrakeMode = wantsBrakeMode;
            mIntakeMotor.setIdleMode( IdleMode.kBrake );
            mLogger.info( "Neutral mode set to: [Brake]" );

        } else if ( !wantsBrakeMode && mIsBrakeMode ) {
            mIsBrakeMode = wantsBrakeMode;
            mIntakeMotor.setIdleMode( IdleMode.kCoast );
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

    public Intake ( WPI_TalonSRX intakeMotor ) {
  
        // Set the hardware
        mIntakeMotor = intakeMotor;
        //mJoystick = joystick;

        // Set the hardware states
        mIsBrakeMode = false;
        SetBrakeMode( true );
        mIsOuttake = false;
        SetOuttake( true );
    }

    public static Intake create () {
        WPI_TalonSRX master = TalonSRX.createTalonSRXWithEncoder( new WPI_TalonSRX( INTAKE.MASTER_ID) );
        return new Intake( intakeMotor );
    }

    @Override
    public void periodic () {

    }

}