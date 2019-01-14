package frc.robot;

import java.io.FileNotFoundException;
import java.io.PrintWriter;

import Obj.MessageLevel;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

/**
 * Summary: this class is designed to access a very powerful logging system very
 * easily. This class is a singleton. It takes the sender, message, messageLevel
 * from anyone and logs it through two ways: (sysout print line && saving log to
 * log.csv). CSV writer flushing is done automatically every 0.5 seconds using
 * the wpi notifier.
 * 
 * To use: call getInstance once, and simply call any of the log functions to
 * log info
 */

public class Logger {

	private static Logger instance;
	// private static Notifier notifier;

	PrintWriter writer = null;

	public static Logger getInstance() {
		if (instance == null)
			instance = new Logger();
		return instance;
	}

	private Logger() {
		try {
			String name = DriverStation.getInstance().getEventName() + DriverStation.getInstance().getMatchType()
					+ "Match" + DriverStation.getInstance().getMatchNumber() + ".txt";
			// writer = new PrintWriter("/home/lvuser/" + name + ".csv");
			writer = new PrintWriter("/home/lvuser/Log.csv");
		} catch (FileNotFoundException e) {
			e.printStackTrace();
			System.out.println("Logger init failed: file not found.");
		}
		System.out.println("Log inited");
		writer.println("-----,-----------,-----------,--------------");
		writer.println("MessageLevel, Title, Timestamp, Message");

		// notifier = new Notifier(() -> {
		// flush();
		// System.out.println("flushed");
		// });
		// notifier.startPeriodic(500);
	}

	public void log(Object message) {
		log(message, "Default", MessageLevel.Info);
	}

	public void log(Object message, String sender, MessageLevel messageLevel) {
		logData(messageLevel.name() + ", \"" + sender + "\", " + Timer.getFPGATimestamp() + ", \"" + message + "\"");
	}

	/**
	 * writer.println(data); puts the data into the printWriter buffer, which means
	 * the data is not saved in the file. Use flush to apply buffer to file.
	 */
	private void logData(String data) {
		// System.out.println(data);
		writer.println(data);
		flush();
	}

	/**
	 * writer.flush(); writes (flushes) the data in the buffer into the file. The
	 * reason flush and writer are separated is to increase performance (made
	 * possible by the buffer system).
	 * 
	 * Therefore, flush is called at 2Hz by its own wpi notifier timer
	 */
	public synchronized void flush() {
		if (writer != null) {
			writer.flush();
		}
	}
}
