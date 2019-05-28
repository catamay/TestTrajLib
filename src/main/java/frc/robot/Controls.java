/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import lib.controls.*;
import lib.enums.GameState;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class Controls{
  private Joystick Driver, Operator;
  private TimedRumble driverRumble, operatorRumble;
  
  private static Controls mInstance;

  public synchronized static Controls getInstance() {
    if (mInstance == null) {
        mInstance = new Controls();
    }
    return mInstance;
}

  private Controls(){
    Driver = new Joystick(0);
    Operator = new Joystick(1);
    driverRumble = new TimedRumble(Driver, 0, 0);
    operatorRumble = new TimedRumble(Operator, 0, 0);

  }
  public GameState Init(){
    driverRumble = new TimedRumble(Driver, 1, 1);
    Operator.setRumble(RumbleType.kLeftRumble, 1);
    operatorRumble = new TimedRumble(Operator, 1, 1);
    updateRumble();
    return GameState.kInit;
  }

  public void updateRumble() {
    driverRumble.update();
    operatorRumble.update();
}

  public enum AxisType{
    Y, X, Twist, Throttle
  }
  /**
   * This is a weird method I made as a way to explain enums, it works, but it feels a little clunky
   * @param direction The direction (X, Y, Twist, Throttle) of the Joystick
   * @return a double value of the Joystick directional value
   */
  public double getAxis(AxisType direction){
    switch (direction){
      case Y:
        return Driver.getY();
      case X:
        return Driver.getX();
      case Twist:
        return Driver.getTwist();
      case Throttle: 
        return Driver.getThrottle();
      default:
        return Driver.getThrottle();
    }
  }
  
  public double getThrottle() {
    return -Driver.getRawAxis(Xbox360.AXIS_LEFT_Y);
}    

public double getCurvature() {
    return Driver.getRawAxis(Xbox360.AXIS_RIGHT_X) * 0.82;
}
  
  
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());


}
