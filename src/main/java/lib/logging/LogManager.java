/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package lib.logging;

import java.util.HashMap;

/**
 * A system in which to manage all logs going into the Robot.java class in order to allow all logs to be passed through a single method
 */
public class LogManager{
    private HashMap<String, LoggableClass> activeLogs;
    private Object lock = new Object();

    private static LogManager mInstance;

    public static LogManager getInstance(){
        if(mInstance == null){
            mInstance = new LogManager();
        }
        return mInstance;
    }

    private LogManager(){
        activeLogs = new HashMap<>();
    }

    public synchronized void addLoggable(LoggableClass Loggable) {
        synchronized (lock) {
            if (Loggable != null && !activeLogs.containsKey(Loggable.getClass().getName())) {
                activeLogs.put(Loggable.getClass().getName(), Loggable);
            }
        }
    }

    /**
     * Gets the logPeriodicIO() method from each logger
     */
    public void logPeriodicIO(){
        activeLogs.forEach((k, v) -> v.logPeriodicIO());
    }

    /**
     * Get the loggers out of the created hash maps
     * @return the active loggers
     */
    public HashMap<String, LoggableClass> getLoggers(){
        return activeLogs;
    }
}
