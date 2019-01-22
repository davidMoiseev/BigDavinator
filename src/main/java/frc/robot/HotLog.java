package frc.robot;

import java.io.File;
import java.io.FileWriter;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.LinkedHashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.Timer;

/**
 * Add your docs here.
 */
public class HotLog {
    public static final String LOGS_DIRECTORY = "/home/lvuser/logs/";

    public static final Date LoggerStartDate = new Date();

    public static final String DELIMITER = "\t";

    private static Map<String, String> logMap = new LinkedHashMap<>();

    private static boolean changed = false;

    public static void LogValue(String key, Integer value) {
        LogValue(key, Double.valueOf(value));
    }

    public static void LogValue(String key, Double value) {
        String stringValue = " ";
        try {
            stringValue = String.valueOf(value);
        } catch (Exception ignored) {
        }
        LogValue(key, stringValue);
    }

    public static void LogValue(String key, String value) {
        if (logMap.keySet().contains(key))
        {
            logMap.put(key, value);
            changed = true;
        }
    }

    public static void Setup(String... valsToLog) {
        logMap = new LinkedHashMap<>();
        StringBuilder s = new StringBuilder();
        s.append("TimeStep").append(DELIMITER);
        for (int i = 0; i < valsToLog.length; ++i) {
            logMap.put(valsToLog[i], "");
            s.append(valsToLog[i]).append(DELIMITER);
        }
        s.append("\n");
        String output = s.toString();
        System.out.println(output);

        try {
            String fileName = LOGS_DIRECTORY + new SimpleDateFormat("yyyy-mm-dd hh:mm:ss").format(LoggerStartDate)
                    + ".txt";

            File f = new File(fileName);
            if (!f.exists()) {
                new File(LOGS_DIRECTORY).mkdirs();
                f.createNewFile();
            }

            FileWriter writer = new FileWriter(f);
            writer.append(output);
            writer.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
        timer.start();
    }

    private static Timer timer = new Timer();
    public static void WriteToFile() {
        if (!changed)
            return;
        double timeStep = timer.get();
        timer.reset();
        try {
            StringBuilder s = new StringBuilder();
            s.append(String.valueOf(timeStep)).append(DELIMITER);

            for (Map.Entry<String, String> entry : logMap.entrySet()) {
                s.append(entry.getValue()).append(DELIMITER);
                entry.setValue("");
            }
            s.append("\n");

            String fileName = LOGS_DIRECTORY + new SimpleDateFormat("yyyy-mm-dd hh:mm:ss").format(LoggerStartDate)
                    + ".txt";

            File f = new File(fileName);
            if (!f.exists()) {
                new File(LOGS_DIRECTORY).mkdirs();
                f.createNewFile();
            }

            FileWriter writer = new FileWriter(f, true);
            writer.append(s.toString());
            writer.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
        changed = false;
    }
}
