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

//Copy of LogToFileUtils with the express purpose of writing statistics files
//These files will follow a strict format so correct usage is important
//Files may be read by Python or some other graphing utility

public class StatFileUtils {
    private static Context mContext;
    private static StatFileUtils instance;
    private static File stat_file;
    private static SimpleDateFormat logSDF = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss");
    private static final int LOG_MAX_SIZE = 10 * 1024 * 1024;
    private static String tag;

    private static final String class_tag = "StatFileUtils: ";

    public static void init(Context context) {
        tag = class_tag;
        Log.i(class_tag, "Initialisation called");
        if (null == mContext || null == instance || null == stat_file || !stat_file.exists()) {
            mContext = context; //Initialiser's context
            instance = new StatFileUtils(); //Static construction
            stat_file = getStatFile(); //
            Log.i(class_tag, "stat_file path is: " + stat_file.getPath());

            // File size
            long stat_file_size = getFileSize(stat_file);
            Log.d(class_tag, "Stat max size is: " + Formatter.formatFileSize(context, LOG_MAX_SIZE));
            Log.i(class_tag, "Stat file size is currently: " + Formatter.formatFileSize(context, stat_file_size));

            // recreate the file if the size is too big
            if (LOG_MAX_SIZE < stat_file_size) {
                resetStatFile();
            }

        } else {
            Log.i(class_tag, "StatFileUtils initialised successfully");
        }
    }


    public static void write(String task_code, String info_str, Object str) {
        // Whether the initialisation is successful.
        if (null == mContext || null == instance || null == stat_file || !stat_file.exists()) {
            Log.e(class_tag, "Initialisation failure!!!");
            return;
        }

        //"new" is a keyword for delimiting instances; new should be printed on its own
        if ( info_str.equals("") ){ info_str = "none"; }
        if ( task_code.equals("") ){ task_code = "none"; }

        String logStr = "[" + logSDF.format(new java.util.Date()) + " " + task_code + ":" + info_str + "] " + str.toString() + ";\n";


        Log.i(class_tag, logStr);

        //Write to file - three styles of write:
        //[yyyy:mm:dd hh:mm:ss TASK:INFO_STR] new;
        //[yyyy:mm:dd hh:mm:ss TASK:INFO_STR] (-t title)? {comma, separated, data};
        //[yyyy:mm:dd hh:mm:ss TASK:INFO_STR] -- comment;

        try {
            BufferedWriter bw = new BufferedWriter(new FileWriter(stat_file, true));
            bw.write(logStr);
            bw.write("\r\n");
            bw.flush();
        } catch (Exception e) {
            Log.e(class_tag, "Write failure !!! " + e.toString());
        }
    }

    private static void resetStatFile() {
        Log.i(class_tag, "Reset Log File ... ");
        // Create Log file, delete the old one
        File last_stat_file = new File(stat_file.getParent() + "/last_stat_file.txt");
        if (last_stat_file.exists()) {
            last_stat_file.delete();
        }
        // Rename the file to lastLog.txt
        stat_file.renameTo(last_stat_file);
        // Create New File
        try {
            stat_file.createNewFile();
        } catch (Exception e) {
            Log.e(class_tag, "Create log file failure !!! " + e.toString());
        }
    }

     private static long getFileSize(File file) {
        long size = 0;
        if (file.exists()) {
            try {
                FileInputStream fis = new FileInputStream(file);
                size = fis.available();
            } catch (Exception e) {
                Log.e(class_tag, e.toString());
            }
        }
        return size;
    }

    private static File getStatFile() {
        File file;
        // Check if SD card is mounted
        if (Environment.getExternalStorageState().equals(Environment.MEDIA_MOUNTED)) {
            // SD\Android\data\Package Name\files\Log\logs.txt
            file = new File(mContext.getExternalFilesDir("Stat").getPath() + "/");
        } else {
            // If there is no SD card or external storage, then use the internal storage
            // /data/data/packagename/files/Statistics/stats.txt
            // \data\data\包名\files\Log\logs.txt
            file = new File(mContext.getFilesDir().getPath() + "/Statistics/");
        }
        // Create the folder if it does not exist
        if (!file.exists()) {
            file.mkdir();
            Log.i(class_tag, "New directory created");
        }
        File stat_file = new File(file.getPath() + "/reaction_times.txt");
        if (!stat_file.exists()) {
            try {
                stat_file.createNewFile();
            } catch (Exception e) {
                Log.e(class_tag, "Create stat file failure!!! " + e.toString());
            }
        }
        return stat_file; //Return the File representing the statistics file
    }


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
