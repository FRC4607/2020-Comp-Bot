package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.ControlType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANS parkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class Hood extends SubsystemBase {

    public static enum controlType {
        kInitState,
        kSeekingState,
        kHoldingState, 
        kFailState, 
        kSmartMotion
    }

    // Hardware
    private final CANSparkMax mHoodMotor;

    //Control State
    private controlType mControlState;

    private CANEncoder mHoodEncoder;

    /****************************************************************************************************************************** 
     ** GETTERS
    ******************************************************************************************************************************/
    public controlType getControlState() {
        return mControlState;
    }

  /****************************************************************************************************************************** 
  ** SET SMART MOTION OUTPUT
  ******************************************************************************************************************************/
    public void SmartMotion {
        if (mControlState != controlType.kSmartMotion) {
            mControlState = controlType.kSmartMotion;
            mHoodMotor.selectProfileSlot(Constants.SMART_MOTION_ID);
        }

        mEncoderPositionTicks = getEncoderPositionTicks();
        
        if (mEncoderPositionTicks < targetPositionTicks) {
            mHoodFFGravityComponent = Math.cos(Math.toRadians(getAngle(mEncoderPositionTicks)));
        } else {
            mHoodFFGravityComponent = 0.0;
    }
    // mLogger.info("Encoder position: {}, target position: {}, Feed Forward: {}", mEncoderPositionTicks, targetPositionTicks, mWristFFGravityComponent);
    mHoodMotor.set(ControlType.IntState, targetPositionTicks);
  }

  public void setOpenOutput(double zHood) {
    mHoodMotor.set(zHood);
  }
  

    // if(mode) {
    //     setPoint = SmartDashboard.getNumber("Set Velocity", 0);
    //     m_pidController.setReference(setPoint, ControlType.kVelocity);
    //     processVariable = m_encoder.getVelocity();
    //   } else {
    //     setPoint = SmartDashboard.getNumber("Set Position", 0);

    // Hardware states
    private boolean mIsBrakeMode;

    // Logging
    private final Logger mLogger = LoggerFactory.getLogger( Hood.class );

    public void SetBrakeMode ( boolean wantsBrakeMode ) {
        if (wantsBrakeMode && !mIsBrakeMode) {
            mIsBrakeMode = wantsBrakeMode;
            mHoodMotor.setIdleMode(IdleMode.kBrake);
            mLogger.info("Neutral mode set to: [Brake]");

        } else if (!wantsBrakeMode && mIsBrakeMode) {
            mIsBrakeMode = wantsBrakeMode;
            mHoodMotor.setIdleMode(IdleMode.kCoast);
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

    public Hood ( CANSparkMax hoodMotor ) {
  
        // Set the hardware
        mHoodMotor = hoodMotor;

        // Set the hardware states
        mIsBrakeMode = false;
        SetBrakeMode( true );
    }

    // uses rev through bore encoder

    public static Hood create () {
        // Talon's and Victor's go through a custom wrapper for creation
        CANSparkMax hoodMotor =  new CANSparkMax( Constants.HOOD_MOTOR_ID, MotorType.kBrushless );
        return new Hood( hoodMotor );
    }

    public CANEncoder getAlternateEncoder() {
		return new CANEncoder(mHoodMotor, AlternateEncoderType.kQuadrature, 0);
	}


    @Override
    public void periodic () {

    }

}

