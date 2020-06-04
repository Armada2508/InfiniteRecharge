package frc.lib.util;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

public class LogUtil {

	// RFC2822
    private static SimpleDateFormat kDateFormat = new SimpleDateFormat("EEE', 'dd' 'MMM' 'yyyy' 'HH:mm:ss' 'Z", Locale.US);
    
	private static SimpleDateFormat kSessionNameFormat = new SimpleDateFormat("dd-MMM-yyyy-HH:mm:ss", Locale.US);

	public static String getTimestamp() {
		return kDateFormat.format(new Date());
	}

	public static String genSessionName() {
		return kSessionNameFormat.format(new Date());
	}

	public static String fromBool(boolean in) {
		// Saves on disk usage by not including decimals
		return in ? "1" : "0";
	}
}