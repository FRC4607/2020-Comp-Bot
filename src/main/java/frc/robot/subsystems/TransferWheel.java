package frc.robot.subsystems;

import frc.robot.Constants.TRANSFER_WHEEL;
import frc.robot.lib.drivers.Photoeye;
import frc.robot.lib.drivers.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class TransferWheel extends SubsystemBase {

    // Hardware
    private final CANSparkMax mMaster;
    //private final Photoeye mIndexerPhotoeye = new Photoeye( TRANSFER_WHEEL.PHOTOEYE_DIO_CHANNEL );

    // Logging
    private final Logger mLogger = LoggerFactory.getLogger( TransferWheel.class );

    public TransferWheel ( CANSparkMax master ) {
        mMaster = master;
    }

    public static TransferWheel create () {
        CANSparkMax master =  SparkMax.CreateSparkMax( new CANSparkMax( TRANSFER_WHEEL.MASTER_ID, MotorType.kBrushless ) );
        return new TransferWheel( master );
    }

    @Override
    public void periodic () {

    }

}

