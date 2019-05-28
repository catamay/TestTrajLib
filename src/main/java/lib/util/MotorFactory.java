/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package lib.util;

import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.can.*;
/**
 * Creates Spark Max objects and configures all the parameters we care about to factory defaults. Closed-loop and sensor
 * parameters are not set, as these are expected to be set by the application.
 */
public class MotorFactory {


    //Master Motor Controllers used (You can add whatever, this is just to help keep object oriented conventions)
    /**
     * Creates a Brushless Neo object for a subsystem using a Spark Max motor controller
     * @param id Motor Controller ID
     * @return CANSparkMax object to be used in your subsystem code
     */
    public static CANSparkMax createBrushlessNeo(int id){
        CANSparkMax spark = new CANSparkMax(id, MotorType.kBrushless);
        spark.setSmartCurrentLimit(60); //This can be set to whatever, but it's just the peak current limit value 
        return spark;
    }
    /**
     * Creates a Brushed motor object for a subsystem using a Spark Max motor controller
     * @param id Motor Controller ID
     * @return CANSparkMax object to be used in your subsystem code
     */
    public static CANSparkMax createBrushedMotor(int id){
        CANSparkMax spark = new CANSparkMax(id, MotorType.kBrushed);
        spark.setSmartCurrentLimit(60);
        return spark;
    }
    /**
     * Creates a motor object for a subsystem using a TalonSRX motor controller
     * @param id Motor Controller ID
     * @return TalonSRX object to be used in your subsystem code
     */
    public static TalonSRX createSRXMotor(int id){
        TalonSRX talon = new TalonSRX(id);
        talon.configPeakCurrentLimit(60);
        return talon;
    }
    /**
     * Creates a motor object for a subsystem using a VictorSPX motor controller
     * @param id Motor Controller ID
     * @return VictorSPX object to be used in your subsystem code
     */
    public static VictorSPX createSPXMotor(int id){
        VictorSPX victor = new VictorSPX(id);
        return victor;
    }

    //Slave Motor Controllers used (You can add whatever else required)

    /**
     * Creates a slaved motor object for a subsystem using a Spark Max motor controller
     * @param id Motor Controller ID
     * @param masterSpark
     * @return CANSparkMax slave object to be used in your subsystem code
     */
    public static CANSparkMax createSlavedSpark(int id, CANSparkMax masterSpark){
        CANSparkMax spark = new CANSparkMax(id, MotorType.kBrushed);
        spark.follow(masterSpark);
        return spark;
    }
    /**
     * Creates a slaved motor object for a subsystem using a TalonSRX motor controller
     * @param id Motor Controller ID
     * @param masterController
     * @return TalonSRX slave object to be used in your subsystem code
     */
    public static TalonSRX createSlavedSRX(int id, IMotorController masterController){
        TalonSRX talon = new TalonSRX(id);
        talon.follow(masterController);
        return talon;
    }
    /**
     * Creates a slaved motor object for a subsystem using a VictorSPX motor controller
     * @param id Motor Controller ID
     * @param masterController
     * @return VictorSPX slave object to be used in your subsystem code
     */
    public static VictorSPX createSlavedSPX(int id, IMotorController masterController){
        VictorSPX victor = new VictorSPX(id);
        victor.follow(masterController);
        return victor;
    }

}
