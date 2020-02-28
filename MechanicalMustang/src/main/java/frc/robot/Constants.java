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
        public static final int kBallLift = 4;
        public static final int kBallRoller = 5;
        public static final int kCrawl = 6;
        public static final int kClimb = 7;
    }

    public static final class MotorSpeeds {
        public static final double kFlipUp = 0.5;
        public static final double kFlipDown = -0.5;
        public static final double kRotateWheel = 0.5;
        public static final double kLiftUp = 0.5;
        public static final double kLowerDown = -0.5;
        public static final double kRollIn = 0.5;
        public static final double kRollOut = -0.5;
        public static final double kWinchLift = 0.5;
        public static final double kWinchExtend = -0.5;
    }

    public static final class DIO {
        public static final int kFlipperMotor = 4;
        public static final int kLiftMotor = 5;
        public static final int[] kLeftEncoderPorts = new int[]{0, 1};
        public static final int[] kRightEncoderPorts = new int[]{2, 3};
        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = false;

        public static final int kEncoderCPR = 1024;
        public static final double kEncoderDistancePerPulse = (6 * Math.PI) / (double) kEncoderCPR;
    }

    public static final class MotorCounts {
        public static final int kFlipUp = 30;
        public static final int kFlipDown = 30;
    }

    public static final class driveStick {
        public static final int kDriveStickPort = 0;
        public static final int kYAxis = 1;
        public static final int kXAxis = 0;
        public static final int kRotateAxis = 2;
        public static final int kDriveStickTrigger = 1;
        public static final int kDriveStickButton2 = 2;
        public static final int kDriveStickButton3 = 3;
        public static final int kDriveStickButton4 = 4;
    }

    public static final class commandStick {
        public static final int kCommandStickPort = 1;
        public static final int kButtonA = 1;
        public static final int kButtonB = 2;
        public static final int kButtonX = 3;
        public static final int kButtonY = 4;
        public static final int kButtonLB = 5;
        public static final int kButtonRB = 6;
        public static final int kButtonBack = 7;
        public static final int kButtonStart = 8;
        //public static final int kFlipUp = 4; //button number for flip up
        //public static final int kFlipDown = 1; // button number for flip down
    }
    
}

