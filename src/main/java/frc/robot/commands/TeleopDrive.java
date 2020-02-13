package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class TeleopDrive extends CommandBase {
    @SuppressWarnings( { "PMD.UnusedPrivateField", "PMD.SingularField" } )
    private final Drivetrain mDrivetrain;
    private final Intake mIntake;
    private final Joystick mIntakeForward;
    private final Joystick mIntakeBackward;
    private final XboxController mDriverXbox;
    private final XboxController mOperatorXbox;


    @Override
    public void initialize () {}

    @Override
    public void execute() {
        // Arcade drive driven off of the left Xbox joystick only 
        if ( mDrivetrain.IsReversed() ) {
            mDrivetrain.mDifferentialDrive.arcadeDrive( mDriverXbox.getY( Hand.kLeft ), mDriverXbox.getX( Hand.kLeft ) );
        } else {
            mDrivetrain.mDifferentialDrive.arcadeDrive( mDriverXbox.getY( Hand.kLeft ), -mDriverXbox.getX( Hand.kLeft ) );
        }

        if ( mIntake.IsIntake() ) {
            mIntake( mOperatorXbox.getY( Trigger.kForward ) );
        } else if ( mIntake.IsOuttake() ) {
            mIntake( mOperatorXbox.getY( Trigger.kLeft ) );
        }

    }

    @Override
    public void end ( boolean interrupted ) {}

    @Override
    public boolean isFinished () {
      return false;
    }

    public TeleopDrive ( Drivetrain drivetrain, XboxController driverXbox, Intake intake, XboxController operatorXbox, Joystick intakeForward, Joystick intakeBackward ) {
        mDrivetrain = drivetrain;
        mDriverXbox = driverXbox;
        mIntake = intake;
        mIntakeForward = intakeForward;
        mIntakeBackward = intakeBackward;
        mOperatorXbox = operatorXbox;
        addRequirements( mDrivetrain );
    }

}

