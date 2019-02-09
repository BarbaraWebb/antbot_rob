package insectsrobotics.imagemaipulations.Media;
import java.io.File;
import java.io.FileOutputStream;
import java.util.LinkedList;
import java.util.Queue;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.ejml.simple.SimpleMatrix;

import android.os.AsyncTask;
import android.os.Environment;
import android.graphics.Bitmap;
import android.util.Log;
import android.os.SystemClock;

//
// Class to provide video recording facilities for processed image frames. If the frames haven't been
// processed, use Android native recording functionality.
//
public class VideoRecorder {
    //
    // Recording thread; this should be run in the background, constantly trying to empty the
    // frame Buffer. This should only be running if we have frames in the buffer
    // and recording is desired.
    //
    Runnable recorder = new Runnable() {
        @Override
        public void run(){
            while(true){
                //
                // If we get here, either there were frames in the buffer or a
                // new frame has woken it up.
                //
                if (!frameBuffer.isEmpty()) {
                    storeFrame(frameBuffer.removeFirst());
                    frameId++; // Increment frame counter
                }
            }
        }
    };

    private int frameId = 0;
    private LinkedList<Mat> frameBuffer = new LinkedList<Mat>();
    private Thread recording = new Thread(recorder);
    private String tag = "_REC_";
    File sd = new File(Environment.getExternalStorageDirectory(),"/DCIM/VideoFrames/");
    boolean success = true;

    // Interrupt the recording if desired (at the end of a run)
    public void stopRecording(){ recording.interrupt(); }

    // Start recording
    public void startRecording(){
        //
        // If the Frame directory exists, delete it and recreate to guarantee an overwrite.
        //
        if (sd.exists()){ sd.delete(); }
        success = sd.mkdir();

        recording.start();
    }

    //
    // Take on new frame; accounting for the fact that the save will be slower than the
    // time between frames. Add to the buffer and wake the thread.
    //
    public void recordFrame(Mat frame){
        frameBuffer.add(frame);
    }

    //
    // Method to save the frames on the DCIM/VideoRecordings/ directory
    // modified from a similar method in Util.
    //
    private void storeFrame(Mat image)
    {
        Bitmap bmp = null;

        // Generate filename as the number of the frame recorded.
        String filename = String.format("frame_%05d", frameId);

        try
        {
            bmp = Bitmap.createBitmap(image.cols(), image.rows(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(image, bmp);
        }
        catch (Exception e)
        {
            Log.d(tag, e.getMessage());
        }

        FileOutputStream out = null;
        filename += ".png";

        if (success) {
            File dest = new File(sd, filename);
            Log.d(tag, dest.getAbsolutePath());
            try {
                out = new FileOutputStream(dest);
                bmp.compress(Bitmap.CompressFormat.PNG, 100, out); // bmp is your Bitmap instance
                // PNG is a lossless format, the compression factor (100) is ignored

            } catch (Exception e) {
                e.printStackTrace();
                Log.d(tag, e.getMessage());
            } finally {
                try {
                    if (out != null) {
                        out.close();
                        Log.d(tag, "OK!!");
                    }
                } catch (Exception e) {
                    Log.d(tag, e.getMessage() + "Error");
                    e.printStackTrace();
                }
            }
        }
    }
}
