/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.XboxController;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
// public class Drive_Turn_To_Setpoint extends PIDCommand {
  /**
   * Creates a new Drive_Turn_To_Setpoint.
   */
    // private final Drivetrain mDrivetrain;
    // private final XboxController mDriverXbox;
    // private double mPositionTolerance = 2.0f;
    // private double m_minimumInput;
    // private double m_maximumInput;
    // private double m_inputRange;
    // private static final double mkP = 0.015;
    // private static final double mkI = 0.0;
    // private static final double mkD = 0.02;

  // public Drive_Turn_To_Setpoint( Drivetrain drivetrain, XboxController driverXbox, double setpoint ) {
    // super(
    //     // The controller that the command will use
    //     new PIDController(mkP, mkI, mkD),
    //     // This should return the measurement
    //     () -> drivetrain.getHeading(),
    //     // This should return the setpoint (can also be a constant)
    //     () -> setpoint,
    //     // This uses the output
    //     output -> {
    //       drivetrain.mDifferentialDrive.arcadeDrive( 0, output );

    //       // Use the output here
    //     });

        // getController().setTolerance(mPositionTolerance);
        
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
//     mDrivetrain = drivetrain;
//     mDriverXbox = driverXbox;
//     addRequirements( mDrivetrain );
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return getController().atSetpoint();
//   }

//    // Called just before this Command runs the first time
//    @Override
//    public void initialize() {
//      // Get everything in a safe starting state.
//      //mDrivetrain.reset();
//      super.initialize();
//    }

// }
