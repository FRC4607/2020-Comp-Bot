package frc.robot.subsystems;

import frc.robot.Constants.HOPPER;
import frc.robot.Constants.CURRENT_LIMIT;
import frc.robot.lib.drivers.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class Hopper extends SubsystemBase {

    // Hardware
    private CANSparkMax mMaster; 

    // Logging
    private final Logger mLogger = LoggerFactory.getLogger( Hopper.class );

    // spin hopper sending power cells into indexer
    public void Spin() {
        mMaster.set( HOPPER.SPEED );
    }

    // spin hopper backward away from indexer
    public void SpinBack() {
        mMaster.set( -HOPPER.SPEED );
    }

    public void Stop() {
        mMaster.set( 0.0 );
    }

    public Hopper ( CANSparkMax master ) {
        mMaster = master;
        // Current limiting
     mMaster.setSmartCurrentLimit( 6, 6, CURRENT_LIMIT.LONG_CAN_TIMEOUT_MS );
    }

    public static Hopper create () {
        CANSparkMax master =  SparkMax.CreateSparkMax( new CANSparkMax( HOPPER.MASTER_ID, MotorType.kBrushless ) );
        return new Hopper( master );
    }

    @Override
    public void periodic () {

    }

}

