// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// /**
//  * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
//  * constants.  This class should not be used for any other purpose.  All constants should be
//  * declared globally (i.e. public static).  Do not put anything functional in this class.
//  *
//  * <p>It is advised to statically import this class (or one of its inner classes) wherever the
//  * constants are needed, to reduce verbosity.
//  */
package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public final class Constants {

  // Joysticks
  public static int kOperatorJoystickId  = 1;

  // Flywheel
  public static int kRightFlyWheelLeader  = 11;
  public static int kLeftFlyWheelFollower = 25;
  // public static int kFlyWheelJoystickAxis = 1;

  // hood
  public static int kHood           = 7;
  public static MotorType kHoodType = MotorType.kBrushless;

  // Hopper
  public static int kHopper         = 6;
  public static MotorType kHopperType = MotorType.kBrushless;

  // transfer wheel
  public static int kTransferWheel           = 3;
  public static MotorType kTransferWheelType = MotorType.kBrushless;

}
