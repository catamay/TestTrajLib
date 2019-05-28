/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * I know people don't like constants classes, but by just creating sub classes, variables are visible only where the class is called.
 */
public class Constants {
    public class TestDrive {
        //Motor Controllers
        public static final int kLeftMasterID = 0;
        public static final int kRghtMasterID = 1;
        public static final int kLeftSlaveAID = 2;
        public static final int kLeftSlaveBID = 3;
        public static final int kRghtSlaveAID = 4;
        public static final int kRghtSlaveBID = 5;

        public static final double kQuickStopThreshold = 0.0; 
        public static final double kQuickStopAlpha = 0.0;
        

        //PIDF Constants

    }
}
