/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package lib.logging;


import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * This logger class creates files inside of a required Logs (case sensitive) folder outside of this source folder (It is under .gitignore for organizational purposes)
 */
public class Logger {
	private static DateFormat fileDate = new SimpleDateFormat("yyyyMMdd HHmmss");
	private static DateFormat prntDate = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss");
	static Date dateTime = new Date();
	/**
	 * Clears a log of the same file name described in the constructor (however, because it's a specific date/time, nothing will be removed, but I'll implement a way for the Rio to additionally get these logs)
	 */
	public static void clearLog() {
		try {
			java.lang.Runtime.getRuntime().exec("/bin/rm -f ../TestTrajLib/Logs/"+ fileDate.format(dateTime)+ ".txt");
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	/**
	 * Logs specific data
	 * @param mark is the object (string) that contains the data that you want to have logged
	 * @param isTitle is a boolean that describes whether or not the object is a section title (such as a subsystem name)
	 */
	public static void log(String mark, Boolean isTitle) {
		try (PrintWriter writer = new PrintWriter(new FileWriter("../TestTrajLib/Logs/"+ fileDate.format(dateTime)+ ".txt", true))) {
			if(isTitle){
				writer.print(mark + ":\n");
			}else{
			writer.print(prntDate.format(dateTime) + ": " + mark);
			//System.out.println(mark);
			}

		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	/**
	 * Logs specific data, ignoring the title boolean because we don't always need to care about a title
	 * @param mark is the object (string) that contains the data that you want to have logged
	 */
	private static void log(String mark) {
		log(mark, false);
	}

	/**
	 * Logs specific data, allowing the data not having to be converted to a string inside of the method (it's done in the constructor here: {@link #log(String, Object)})
	 * @param key is the text you want to have logged alongside the data
	 * @param value is the data you want to have logged, it doesn't matter what data is sent through here because it's an object
	 */
	public static void log(String key, Object value) {
		log(key + ": " + value.toString() + "\n");
	}
}