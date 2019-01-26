package frc.robot;

import static org.junit.Assert.assertNotNull;

import java.io.File;
import java.io.FileWriter;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

/**
 * Add your docs here.
 */
public class HotLog {
    public static final String LOGS_DIRECTORY = "/home/lvuser/logs/";

    public static final String DELIMITER = "\t";
    public static final String EMPTY = "";
    public static final Double ROW_TIMEOUT_SECONDS = .02;
    public static final Double LOG_PERIOD_SECONDS = 1.0;

    private static boolean onNewRow = true;
    private static Map<String, String> currentRow = new LinkedHashMap<String, String>();
    private static double currentRowTime;

    private static Notifier logScheduler = new Notifier(LoggerThread::WriteToFile);

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
        if (!currentRow.containsKey(key))
            return;

        if (onNewRow) {
            currentRowTime = Timer.getFPGATimestamp();
        }

        if (!currentRow.get(key).equals(EMPTY)) {
            PushCurrentRow();
            LogValue(key, value);
            return;
        } else
            currentRow.put(key, value);

        if (Timer.getFPGATimestamp() - currentRowTime > ROW_TIMEOUT_SECONDS) {
            PushCurrentRow();
        }
    }

    private static void PushCurrentRow() {
        try {
            LogRow row = new LogRow(currentRow, String.valueOf(currentRowTime));
            LoggerThread.PushToQueue(row);
        } catch (Exception ignored) {
        }
        for (Map.Entry<String, String> entry : currentRow.entrySet())
        {
            entry.setValue(EMPTY);
        }
        onNewRow = true;
    }

    public static void Setup(String... valsToLog) {

        currentRow.clear();
        StringBuilder headerBuilder = new StringBuilder();
        for (int i = 0; i < valsToLog.length; ++i)
        {
            currentRow.put(valsToLog[i], EMPTY);
            headerBuilder.append(valsToLog[i]).append(DELIMITER);
        }
        headerBuilder.append("\n");

        logScheduler.stop();
        LoggerThread.RestartQueue(headerBuilder.toString());
        logScheduler.startPeriodic(LOG_PERIOD_SECONDS);
    }

    private static class LogRow {
        public final Map<String, String> Values;
        public final String TimeStamp;

        public LogRow(Map<String, String> values, String logTimeStamp) {
            this.Values = values;
            this.TimeStamp = logTimeStamp;
        }
    }

    private static class LoggerThread {

        private static FileWriter fileWriter;

        /**
         * Function called only by logging thread
         */
        public static void WriteToFile() {
            try {
                String fileName = LOGS_DIRECTORY + new SimpleDateFormat("yyyy-MM-dd HH:mm:ss").format(GetDate()) + ".txt";

                File f = new File(fileName);
                if (!f.exists()) {
                    new File(LOGS_DIRECTORY).mkdirs();
                    f.createNewFile();

                    if (fileWriter != null)
                        fileWriter.close();

                    fileWriter = new FileWriter(f);
                }

                if (fileWriter == null)
                    fileWriter = new FileWriter(f);
                fileWriter.append(FlushQueue());
            } catch (Exception e) {
                e.printStackTrace();
            }
        }

        /**
         * Log structures shared between threads
         */
        private static List<LogRow> logQueue = new ArrayList<>();
        private static String headerToOutput = "";
        private static Date logDate = new Date();

        private static synchronized String FlushQueue() {
            StringBuilder output = new StringBuilder(headerToOutput);
            for (LogRow row : logQueue) {
                output.append(DELIMITER).append(row.TimeStamp).append(DELIMITER);
                for (Map.Entry<String, String> e : row.Values.entrySet()) {
                    output.append(e.getValue()).append(DELIMITER);
                }
                output.append("\n");
            }
            headerToOutput = "";
            logQueue.clear();
            return output.toString();
        }

        private static synchronized Date GetDate() {
            return logDate;
        }

        public static synchronized void PushToQueue(LogRow row) {
            logQueue.add(row);
        }

        public static synchronized void RestartQueue(String header) {
            headerToOutput = header;
            logDate = new Date();
            logQueue.clear();
        }
    }
}
