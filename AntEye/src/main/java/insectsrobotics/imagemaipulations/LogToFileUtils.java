package insectsrobotics.imagemaipulations;

import android.content.Context;
import android.os.Environment;
import android.text.format.Formatter;
import android.util.Log;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileWriter;
import java.text.SimpleDateFormat;

/**
 * A Library used to ouput log message to file.
 * Can be found here : https://github.com/itgoyo/LogToFile
 * Created by itgoyo on 2017/2/23.
 */

public class LogToFileUtils {
    /**
     * Context Object
     */
    private static Context      mContext;
    /**
     * FileLogUtils Instance
     */
    private static LogToFileUtils instance;
    /**
     * File to save the log
     */
    private static File         logFile;
    /**
     * Time format
     */
    private static       SimpleDateFormat logSDF       = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss");
    /**
     * 日志的最大占用空间 - 单位：字节
     * <p>
     * 注意：为了性能，没有每次写入日志时判断，故日志在写入第二次初始化之前，不会受此变量限制，所以，请注意日志工具类的初始化时间
     * <p>
     * 为了衔接上文，日志超出设定大小后不会被直接删除，而是存储一个副本，所以实际占用空间是两份日志大小
     * <p>
     * 除了第一次超出大小后存为副本外，第二次及以后再次超出大小，则会覆盖副本文件，所以日志文件最多也只有两份
     * <p>
     * 默认10M
     */
    private static final int              LOG_MAX_SIZE = 10 * 1024 * 1024;
    /**
     * User class as the TAG
     */
    private static String tag;

    private static final String MY_TAG = "LogToFileUtils";

    /**
     * Initialise libraries
     *
     * @param context
     */
    public static void init(Context context) {
        Log.i(MY_TAG, "init ...");
        if (null == mContext || null == instance || null == logFile || !logFile.exists()) {
            mContext = context;
            instance = new LogToFileUtils();
            logFile = getLogFile();
            Log.i(MY_TAG, "LogFilePath is: " + logFile.getPath());
            // File size
            long logFileSize = getFileSize(logFile);
            Log.d(MY_TAG, "Log max size is: " + Formatter.formatFileSize(context, LOG_MAX_SIZE));
            Log.i(MY_TAG, "log now size is: " + Formatter.formatFileSize(context, logFileSize));
            // recreate the file if the size is too big
            if (LOG_MAX_SIZE < logFileSize) {
                resetLogFile();
            }
        } else {
            Log.i(MY_TAG, "LogToFileUtils has been init ...");
        }
    }

    /**
     * Write data to the file
     *
     * @param str 需要写入的数据
     */
    public static void write(Object str) {
        // Whether the initialisation is successful.
        if (null == mContext || null == instance || null == logFile || !logFile.exists()) {
            Log.e(MY_TAG, "Initialization failure !!!");
            return;
        }
        String logStr = getFunctionInfo() + " - " + str.toString();
        Log.i(tag, logStr);

        try {
            BufferedWriter bw = new BufferedWriter(new FileWriter(logFile, true));
            bw.write(logStr);
            bw.write("\r\n");
            bw.flush();
        } catch (Exception e) {
            Log.e(tag, "Write failure !!! " + e.toString());
        }
    }

    /**
     * 重置日志文件
     * <p>
     * 若日志文件超过一定大小，则把日志改名为lastLog.txt，然后新日志继续写入日志文件
     * <p>
     * 每次仅保存一个上一份日志，日志文件最多有两份
     * <p/>
     */
    private static void resetLogFile() {
        Log.i(MY_TAG, "Reset Log File ... ");
        // Create Log file, delete the old one
        File lastLogFile = new File(logFile.getParent() + "/lastLog.txt");
        if (lastLogFile.exists()) {
            lastLogFile.delete();
        }
        // Rename the file to lastLog.txt
        logFile.renameTo(lastLogFile);
        // Create New File
        try {
            logFile.createNewFile();
        } catch (Exception e) {
            Log.e(MY_TAG, "Create log file failure !!! " + e.toString());
        }
    }

    /**
     * Fecth the file size
     *
     * @param file
     * @return
     */
    private static long getFileSize(File file) {
        long size = 0;
        if (file.exists()) {
            try {
                FileInputStream fis = new FileInputStream(file);
                size = fis.available();
            } catch (Exception e) {
                Log.e(MY_TAG, e.toString());
            }
        }
        return size;
    }

    /**
     * Acquire the App log file
     *
     * @return app log file
     */
    private static File getLogFile() {
        File file;
        // Whether there is no SD card or external storage
        if (Environment.getExternalStorageState().equals(Environment.MEDIA_MOUNTED)) {
            //
            // SD\Android\data\Package Name\files\Log\logs.txt
            file = new File(mContext.getExternalFilesDir("Log").getPath() + "/");
        } else {
            // If there is no SD card or external storage, then use the internal storage
            // \data\data\包名\files\Log\logs.txt
            file = new File(mContext.getFilesDir().getPath() + "/Log/");
        }
        // Create the folder if does not exist
        if (!file.exists()) {
            file.mkdir();
        }
        File logFile = new File(file.getPath() + "/logs.txt");
        if (!logFile.exists()) {
            try {
                logFile.createNewFile();
            } catch (Exception e) {
                Log.e(MY_TAG, "Create log file failure !!! " + e.toString());
            }
        }
        return logFile;
    }

    /**
     * Current Function Information
     *
     * @return Current Function Information
     */
    private static String getFunctionInfo() {
        StackTraceElement[] sts = Thread.currentThread().getStackTrace();
        if (sts == null) {
            return null;
        }
        for (StackTraceElement st : sts) {
            if (st.isNativeMethod()) {
                continue;
            }
            if (st.getClassName().equals(Thread.class.getName())) {
                continue;
            }
            if (st.getClassName().equals(instance.getClass().getName())) {
                continue;
            }
            tag = st.getFileName();
            return "[" + logSDF.format(new java.util.Date()) + " " + st.getClassName() + " " + st
                    .getMethodName() + " Line:" + st.getLineNumber() + "]";
        }
        return null;
    }

}
