package frc.robot.subsystems;

import frc.robot.Constants.HOPPER;
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

    public Hopper ( CANSparkMax master ) {
        mMaster = master;
    }

    public static Hopper create () {
        CANSparkMax master =  SparkMax.CreateSparkMax( new CANSparkMax( HOPPER.MASTER_ID, MotorType.kBrushless ) );
        return new Hopper( master );
    }

    @Override
    public void periodic () {

    }

}

