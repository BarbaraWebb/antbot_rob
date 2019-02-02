package insectsrobotics.imagemaipulations;

import android.content.Context;
import android.os.Environment;
import android.util.Log;

import org.opencv.core.CvType;
import org.opencv.core.Mat;

import java.io.File;
import java.nio.Buffer;
import java.nio.ByteBuffer;

import insectsrobotics.imagemaipulations.Recording.FFmpegFrameRecorder;
import insectsrobotics.imagemaipulations.Recording.Frame;

//
// Allows a developer to create multiple ffmpeg recording objects to record video data from the
// robot.
//
public class VideoRecorder {
    private FFmpegFrameRecorder recorder;
    private String logTag = "VideoRecorder";
    private int imageWidth;
    private int imageHeight;
    private int frameRate;
    private int channels;
    private String ffmpegLink;

    private Frame videoFrame = null;
    private Context mContext;
    private boolean firstFrame = true;
    private long startTime;
    private int frames = 0;

    public VideoRecorder(int width, int height, int channels, int fps, Context mContext, String filename){
        this.mContext = mContext;
        Log.i(logTag, "VideoRecorder initialisation");

        videoFrame = new Frame(width, height, 8, channels);

        Log.i(logTag, "IplImage creation");

        File videoFile = getVideoFile(filename);
        ffmpegLink = videoFile.getAbsolutePath();

        Log.i(logTag, "VideoRecorder: " +
        ffmpegLink + " imageWidth: " + imageWidth + "imageHeight" + imageHeight);

        recorder = new FFmpegFrameRecorder(ffmpegLink,width,height);
        recorder.setFormat("mp4"); // Default to MP4, but this could be passed as an arg
        recorder.setSampleRate(44100); // Audio sample rate; required but not important
        recorder.setFrameRate(fps);
    }

    //
    // Method to be called for each video frame being recorded
    //
    public void onFrame(byte[] data){
        //
        // If first video frame, and the videoImage is initialised, set the recording start time
        //
        if (firstFrame && (videoFrame != null)){
            firstFrame = false;
            startTime = System.currentTimeMillis();
            try {
                recorder.start();
            } catch (Exception e) {
                e.printStackTrace();
            }

            Log.i(logTag, "First frame received. Started recording.");
        }

        if (videoFrame != null){

            long timestamp = 1000 * (System.currentTimeMillis() - startTime);

            //
            // Put the data into the IplImage byte buffer. Note: to anyone in the future that
            // reads this, there may be a more efficient method of performing the video recording
            // without all of the weird conversions to byte buffers. Look up bytedeco frame
            // converters; may be possible to do this with just Mat objects
            //
            Mat videoMatrix = new Mat(10, 90, CvType.CV_8UC1);
            videoMatrix.put(0,0, data);

            ByteBuffer[] dataBuffer = new ByteBuffer[1];
            dataBuffer[0].put(data);
            videoFrame.image = dataBuffer;

            try{
                // Set timestamp
                recorder.setTimestamp(timestamp);

                // Record the current frame
                recorder.record(videoFrame);

                // Update and log the frame count
                frames++;
                Log.i(logTag, "Wrote Frame: " + frames);

            } catch (FFmpegFrameRecorder.Exception e){
                e.printStackTrace();
            }
        }
    }

    public void stopRecording(){


        if (recorder != null){
            Log.i(logTag, "Recording complete, calling stop and release.");

            try{
                recorder.stop();
                recorder.release();
            } catch (FFmpegFrameRecorder.Exception e){
                e.printStackTrace();
            }

            recorder = null;
        }
    }

    private File getVideoFile(String filename){
        File file;
        // Check if SD card is mounted
        if (Environment.getExternalStorageState().equals(Environment.MEDIA_MOUNTED)) {
            // SD\Android\data\Package Name\files\Recordings\capture.mp4
            file = new File(mContext.getExternalFilesDir("Recordings").getPath() + "/");
        } else {
            // If there is no SD card or external storage, then use the internal storage
            // /data/data/packagename/files/Recordings/capture.mp4
            file = new File(mContext.getFilesDir().getPath() + "/Recordings/");
        }
        // Create the folder if it does not exist
        if (!file.exists()) {
            file.mkdir();
            Log.i(logTag, "New directory created");
        }
        File videoFile = new File(file.getPath() + "/capture.mp4");
        if (!videoFile.exists()) {
            try {
                videoFile.createNewFile();
            } catch (Exception e) {
                Log.e(logTag, "Create recording file failure!!! " + e.toString());
            }
        }
        return videoFile; //Return the File representing the statistics file
    }
}
