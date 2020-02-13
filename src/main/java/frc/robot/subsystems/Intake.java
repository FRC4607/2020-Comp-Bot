package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class Intake extends SubsystemBase {

    // Hardware
    private final CANSparkMax mIntakeMotor;

    // Hardware states
    private boolean mIsBrakeMode;
    private boolean mIsIntake;
    private boolean mIsOuttake;

    // Logging
    private final Logger mLogger = LoggerFactory.getLogger( Intake.class );

    public void SetIntake ( boolean wantsIntake ) {
        if ( wantsIntake != mIsIntake ) {
            mIsIntake = wantsIntake;
            mIntakeMotor.setThrottleChannel( wantsIntake );
            mLogger.info( "Intaking set to: [{}]", mIsIntake );
        }
    }

    public boolean IsIntake () {
        return mIsIntake;
    }

    public void SetOuttake ( boolean wantsOuttake ) {
        if ( wantsOuttake != mIsOuttake ) {
            mIsOuttake = wantsOuttake;
            mIntakeMotor.setInverted( wantsOuttake );
            mLogger.info( "Outtaking set to: [{}]", mIsOuttake );
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

    public Intake ( CANSparkMax intakeMotor ) {
  
        // Set the hardware
        mIntakeMotor = intakeMotor;
        //mJoystick = joystick;

        // Set the hardware states
        mIsBrakeMode = false;
        SetBrakeMode( true );
    }

    public static Intake create () {
        // Talon's and Victor's go through a custom wrapper for creation
        CANSparkMax intakeMotor = new CANSparkMax( Constants.INTAKE_MOTOR_ID, MotorType.kBrushless );
        return new Intake( intakeMotor );
    }

    @Override
    public void periodic () {

    }

}