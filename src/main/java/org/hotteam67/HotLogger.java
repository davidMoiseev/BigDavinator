package org.hotteam67;

import java.io.File;
import java.io.FileWriter;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Date;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

/**
 * Add your docs here.
 */
public class HotLogger {
    public static final String LOGS_DIRECTORY = "/home/lvuser/logs/";

    public static final String DELIMITER = "\t";
    public static final String EMPTY = "";
    public static final Double ROW_TIMEOUT_SECONDS = .02;
    public static final Double LOG_PERIOD_SECONDS = 1.0;
    public static final Integer ALLOWED_LOG_FILES = 10;

    private static boolean onNewRow = true;
    private static Map<String, String> currentRow = new LinkedHashMap<String, String>();
    private static double currentRowTime;

    private static Notifier logScheduler = new Notifier(LogThread::WriteToFile);

    public static void Setup(String... valsToLog) {

        currentRow.clear();
        StringBuilder headerBuilder = new StringBuilder();
        headerBuilder.append("Time Step").append(DELIMITER);
        for (int i = 0; i < valsToLog.length; ++i) {
            currentRow.put(valsToLog[i], EMPTY);
            headerBuilder.append(valsToLog[i]).append(DELIMITER);
        }

        logScheduler.stop();
        LogQueue.RestartQueue(headerBuilder.toString());
        logScheduler.startPeriodic(LOG_PERIOD_SECONDS);
    }

    public static void Log(String key, Integer value) {
        Log(key, Double.valueOf(value));
    }

    public static void Log(String key, Double value) {
        String stringValue = " ";
        try {
            stringValue = String.valueOf(value);
        } catch (Exception ignored) {
        }
        Log(key, stringValue);
    }

    public static void Log(String key, String value) {
        if (!currentRow.containsKey(key))
            return;

        if (onNewRow) {
            currentRowTime = Timer.getFPGATimestamp();
        }

        if (!currentRow.get(key).equals(EMPTY)) {
            PushCurrentRow();
            Log(key, value);
            return;
        } else
            currentRow.put(key, value);

        if (Timer.getFPGATimestamp() - currentRowTime > ROW_TIMEOUT_SECONDS) {
            PushCurrentRow();
        }
    }

    private static void PushCurrentRow() {
        try {
            LinkedHashMap<String, String> newMap = new LinkedHashMap<>();
            for (Map.Entry<String, String> currentEntry : currentRow.entrySet()) {
                newMap.put(currentEntry.getKey(), currentEntry.getValue());
            }
            LogRow row = new LogRow(newMap, String.valueOf(currentRowTime));
            LogQueue.PushToQueue(row);
        } catch (Exception ignored) {
        }
        for (Map.Entry<String, String> entry : currentRow.entrySet()) {
            entry.setValue(EMPTY);
        }
        onNewRow = true;
    }

    private static class LogRow {
        public final Map<String, String> Values;
        public final String TimeStamp;

        public LogRow(Map<String, String> values, String logTimeStamp) {
            this.Values = values;
            this.TimeStamp = logTimeStamp;
        }
    }

    private static class LogThread {
        private static FileWriter fileWriter = null;

        /**
         * Function called only by logging thread
         */
        public static synchronized void WriteToFile() {
            try {
                String fileName = LOGS_DIRECTORY
                        + new SimpleDateFormat("yyyy-MM-dd HH:mm:ss").format(LogQueue.GetDate()) + ".txt";

                File f = new File(fileName);
                if (!f.exists()) {
                    File directory = new File(LOGS_DIRECTORY);

                    directory.mkdirs();
                    File[] files = directory.listFiles();

                    if (files.length > ALLOWED_LOG_FILES) {
                        Arrays.sort(files, new Comparator<File>() {

                            @Override
                            public int compare(File arg0, File arg1) {
                                if (arg0.lastModified() < arg1.lastModified())
                                    return 1;
                                else if (arg0.lastModified() > arg1.lastModified())
                                    return -1;
                                else
                                    return 0;
                            }
                        });
                        for (int i = ALLOWED_LOG_FILES - 1; i < files.length; ++i) {
                            files[i].delete();
                        }
                    }

                    f.createNewFile();
                    if (fileWriter != null)
                        fileWriter.close();

                    fileWriter = new FileWriter(f);
                }

                if (fileWriter == null)
                    fileWriter = new FileWriter(f);
                fileWriter.append(LogQueue.FlushQueue());
                fileWriter.flush();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }

        public static synchronized void CloseStream() {
            if (fileWriter != null)
                try {
                    fileWriter.close();
                } catch (Exception e) {
                    e.printStackTrace();
                }
        }
    }

    public static void CloseStream() {
        LogThread.CloseStream();
        logScheduler.stop();
    }

    private static class LogQueue {
        /**
         * Log structures shared between threads
         */
        private static List<LogRow> logQueue = new ArrayList<>();
        private static String headerToOutput = "";
        private static Date logDate = new Date();

        private static synchronized String FlushQueue() {
            StringBuilder output = new StringBuilder(headerToOutput);
            output.append("\n");

            for (int i = 0; i < logQueue.size(); ++i) {
                LogRow row = logQueue.get(i);
                output.append(row.TimeStamp).append(DELIMITER);
                for (Map.Entry<String, String> e : row.Values.entrySet()) {
                    output.append(e.getValue()).append(DELIMITER);
                }
                if (i + 1 < logQueue.size())
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
