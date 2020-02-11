/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public final class Constants {
    public static final class PWM {
        public static final int kLeftDriveMotors = 0;
        public static final int kRightDriveMotors = 1;
        public static final int kFlipperMotor = 2;
        public static final int kRotatorMotor = 3;
    }

    public static final class MotorSpeeds {
        public static final double kFlipUp = 1.0;
        public static final double kFlipDown = -1.0;
    }

    public static final class DIO {
        public static final int kFlipperMotor = 0;
    }

    public static final class MotorCounts {
        public static final int kFlipUp = 40;
        public static final int kFlipDown = 40;
    }

    public static final class driveStick {
        public static final int kDriveStickPort = 0;
        public static final int kYAxis = 1;
        public static final int kXAxis = 0;
        public static final int kRotateAxis = 2;
        public static final int kDriveStickTrigger = 1;
    }

    public static final class commandStick {
        public static final int kCommandStickPort = 1;
        public static final int kFlipUp = 2; //button number for flip up
        public static final int kFlipDown = 3; // button number for flip down
    }
    
}
