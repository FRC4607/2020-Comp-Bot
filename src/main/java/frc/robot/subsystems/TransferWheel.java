package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.slf4j.LoggerFactory;
import org.slf4j.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Flywheel.FlywheelJoystick;

public class TransferWheel extends SubsystemBase {

    public static WPI_TalonSRX mTransferWheel = new WPI_TalonSRX(Constants.kTransferWheel);
    
    private boolean mIsBrakeMode;

    public boolean isBrakeMode() {
      return mIsBrakeMode;
    }
    
    public void ApplyDriveSignal(double throttle) {
      double mThrottle = throttle;
      mTransferWheel.set(mThrottle);
    }

    public void setOpenLoopControl() {
      setBrakeMode(true);
    }

    public TransferWheel(WPI_TalonSRX master) {
      mTransferWheel = master;
  
      mIsBrakeMode = false;
      setBrakeMode(true);
  
      // This is inverted alongside the joystick inputs in order to create working limit switches
      mTransferWheel.setInverted(true);
  
      // Start off in open loop control
      setOpenLoopControl();
    }

    private void setBrakeMode(boolean b) {
    }

    public TransferWheel() {
    }
  
    
    public static void initDefaultSetup() {
    }
		
	}
      
