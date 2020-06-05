package frc.lib.util;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

public class LogUtil {

	// RFC2822
    private static SimpleDateFormat kDateFormat = new SimpleDateFormat("EEE', 'dd' 'MMM' 'yyyy' 'HH:mm:ss' 'Z", Locale.US);
    
	private static SimpleDateFormat kSessionNameFormat = new SimpleDateFormat("dd-MMM-yyyy-HH-mm-ss", Locale.US);

	public static String getTimestamp() {
		return kDateFormat.format(new Date());
	}

	public static String getSessionName() {
		return kSessionNameFormat.format(new Date());
	}

	public static String boolToString(boolean bool) {
		return bool ? "1" : "0";
	}

	public static double boolToDouble(boolean bool) {
		return bool ? 1d : 0d;
	}
}