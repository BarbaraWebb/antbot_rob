package insectsrobotics.imagemaipulations;


import android.app.Activity;
import android.content.ComponentName;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.ServiceConnection;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorManager;
import android.hardware.SensorEventListener;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Environment;
import android.os.IBinder;
import android.os.SystemClock;
import android.util.Log;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.ProgressBar;
import android.widget.TextView;
import android.app.AlertDialog;
import android.content.DialogInterface;

import org.ejml.simple.SimpleMatrix;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.io.FileOutputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import insectrobotics.broadcastlibrary.BroadcastValues;
import insectsrobotics.imagemaipulations.Receiver_and_Broadcaster.Broadcast;
import insectsrobotics.imagemaipulations.Receiver_and_Broadcaster.Receive;
import insectsrobotics.imagemaipulations.NavigationModules.WillshawModule;
import insectsrobotics.imagemaipulations.NavigationModules.RealWillshawModule;

import static java.lang.Thread.sleep;
import static org.opencv.imgproc.Imgproc.GaussianBlur;
import static org.opencv.imgproc.Imgproc.INTER_LANCZOS4;
import static org.opencv.imgproc.Imgproc.INTER_LINEAR;
import static org.opencv.imgproc.Imgproc.arrowedLine;
import static org.opencv.imgproc.Imgproc.goodFeaturesToTrack;
import static org.opencv.imgproc.Imgproc.remap;
import static org.opencv.imgproc.Imgproc.resize;
import static org.opencv.video.Video.calcOpticalFlowFarneback;
import static org.opencv.video.Video.calcOpticalFlowPyrLK;

public class ThreadCode extends MainActivity {

    //
    // Binary MB Learning thread
    //
    Runnable navLearning = new Runnable(){
        @Override
        public void run() {
            try {
                sleep(3000);
            } catch (Exception e) {
                e.printStackTrace();
            }

            mushroom_body = new WillshawModule();
            real_mushroom_body = new RealWillshawModule();

            //Setup the model
            boolean model_not_initialised = true;
            while (model_not_initialised) { //Spin until we can initialise the model
                if (images_access) {
                    //Can't send a matrix, need to send an array
                    Mat matImage = processedDestImage; //get current image
                    byte[] imageArray_tmp = new byte[matImage.height() * matImage.width()]; //Process into an array
                    matImage.get(0, 0, imageArray_tmp);
                    int[] imageArray = new int[imageArray_tmp.length];
                    for (int n = 0; n < imageArray_tmp.length; n++) {
                        imageArray[n] = (int) imageArray_tmp[n] & 0xFF;
                    }

                    //Call the method to construct the network ( for the chosen network type );
                    if (real) {
                        real_mushroom_body.setupLearningAlgorithm(imageArray);
                    } else {
                        mushroom_body.setupLearningAlgorithm(imageArray);
                    }

                    model_not_initialised = false; //Model is now ready, can exit the loop
                }
            }

            //The Willshaw net (Mushroom Body) is now ready to use.

            //Run the CA algorithm (Same as the opticalFlowAvoidance thread)

            boolean avoid = false; //Is the robot currently avoiding an obstacle?
            boolean stop = false; //Halt flag
            boolean initialise = true; //Loop initialisation flag
            int start_time = (int) SystemClock.elapsedRealtime(); //Start time for the thread
            int current_time = (int) SystemClock.elapsedRealtime() - start_time; //Current thread time
            int t_move_start = 0; //Time that a saccade started

            //Accumulation time intervals
            int t_interval_start = current_time; //Start of the time interval
            int t_delta = (int) SystemClock.elapsedRealtime() - current_time; //Time interval

            //Left and right motor speeds
            int lft_speed = 15;
            int rgt_speed = 14;

            //Accumulators for left and right flow
            int dual_accumulator = 0; //Add both values and see if there's significant bias
            int left_accumulator = 0; //Add only left values
            int right_accumulator = 0; //Add only right values
            int accumulation_threshold = 5000; //Threshold for a value to be accumulated (ignore all others)
            int reaction_threshold = 10000; //Value to be met for a reaction to be triggered. (need at most four readings)

            int loop_count = 0;

            //While
            while (!stop && (current_time <= 30000)) {
                try {
                    sleep(600);
                } catch (Exception e) {
                    e.printStackTrace();
                }

                //Print out debug information, catch exceptions
                ca_flow_diff = leftCAFlow - rightCAFlow; //If positive, left detection, else right detection

                //Flow accumulators
                if (ca_flow_diff >= accumulation_threshold) {
                    left_accumulator = left_accumulator + (int) ca_flow_diff;
                } else if (ca_flow_diff <= -accumulation_threshold) {
                    right_accumulator = right_accumulator + Math.abs((int) ca_flow_diff);
                }

                if (Math.abs(ca_flow_diff) >= accumulation_threshold) {
                    dual_accumulator = dual_accumulator + (int) ca_flow_diff;
                }

                //If accumulation time limit reached, reset timer and accumulators
                if (loop_count >= 2/*t_delta >= 2000 */) {
                    left_accumulator = 0;
                    right_accumulator = 0;
                    dual_accumulator = 0;
                    loop_count = 0; //Reset loop counter
                }

                if (images_access){
                    new_image = new SaveImages();
                    new_image.execute(processedDestImage, LEARN_IMAGE);
                }
                    /*boolean image_not_learned = true;
                    Command.go(new double[]{0, 0});
                    try{ sleep(1000); } catch (Exception e){ e.printStackTrace(); }

                    while (image_not_learned) { //Spin until image becomes available
                        //DEBUG NOTE: This shouldn't cause any problems but it's
                        //possible that this delay could cause problems with the CA
                        //by spinning too long. Keep an eye out
                        if (images_access) {
                            Mat matImage = processedDestImage; //get current image
                            byte[] imageArray_tmp = new byte[matImage.height() * matImage.width()]; //Process into an array
                            matImage.get(0, 0, imageArray_tmp);
                            int[] imageArray = new int[imageArray_tmp.length];
                            for (int n = 0; n < imageArray_tmp.length; n++) {
                                imageArray[n] = (int) imageArray_tmp[n] & 0xFF;
                            }

                            //Call the method to construct the network
                            if (real) {
                                real_mushroom_body.learnImage(imageArray);
                            } else {
                                mushroom_body.learnImage(imageArray);
                            }

                            image_not_learned = false; //Image has been learned, we can exit the loop
                        }
                    }
                    Command.go( new double[]{lft_speed, rgt_speed});
                    try { sleep(1000); } catch (Exception e){ e.printStackTrace(); }

                    t_interval_start = (int) SystemClock.elapsedRealtime();*/


                try {
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            debugTextView.setText(String.format(
                                    "Speed: %f \n" +
                                            "Left CA flow: %f \n" +
                                            "Right CA flow: %f \n" +
                                            "Flow Difference: %f \n"
                                    ,
                                    speed,
                                    leftCAFlow,
                                    rightCAFlow,
                                    ca_flow_diff
                            ));
                        }
                    });
                } catch (Exception e) {
                    e.printStackTrace();
                }
                //End printing

                //Initialise if 1st iteration (re-used to reset speeds mid-run)
                if (initialise) {
                    initialise = false;
                    Command.go(new double[]{lft_speed, rgt_speed});
                    try {
                        sleep(1000);
                    } catch (Exception e) {
                        e.printStackTrace();
                    } //Command delay
                    t_interval_start = (int) SystemClock.elapsedRealtime(); //Start the time interval for accumulation
                }

                //Figure out which way to turn
                boolean left = false;
                boolean right = false;

                //Dual decision (contiguous results)
                //if ( dual_accumulator >= reaction_threshold ){ right = true; }
                //else if ( dual_accumulator <= -reaction_threshold ){ left = true; }


                //Uncomment to use the individual method for accumulation.
                //Individual decision
                if (left_accumulator >= reaction_threshold) {
                    right = true;
                } else if (right_accumulator >= reaction_threshold) {
                    left = true;
                }
                //Left accumulator keeps track of left flow and so should trigger a right turn
                //Right accumulator keeps track of right flow and so should trigger a left turn


                if (left && (!avoid)) { //Left turn required
                    Log.i("CA:", "Left turn triggered");

                    Command.go( new double[]{ 0, 0 });
                    try{ sleep(1000); } catch ( Exception e ){ e.printStackTrace(); }

                    try {
                        Command.turnAround(20);
                    } catch (Exception e) {
                        e.printStackTrace();
                    }

                    if (images_access){
                        new_image = new SaveImages();
                        new_image.execute(processedDestImage, LEARN_IMAGE);
                    }

                    /*
                    lft_speed = 10; //Left turn
                    rgt_speed = 100; //Left turn*/
                    initialise = true; //Re-call the go command to trigger the speed change
                    avoid = true; //Flag so we know to ignore this code snippet when avoiding
                    dual_accumulator = 0;
                    left_accumulator = 0;
                    right_accumulator = 0;
                    t_move_start = current_time; //Time the saccade started

                } else if (right && (!avoid)) { //Right turn required
                    Log.i("CA:", "Right turn triggered");

                    Command.go( new double[]{ 0, 0 });
                    try{ sleep(1000); } catch ( Exception e ){ e.printStackTrace(); }



                    try {
                        Command.turnAround(-20);
                    } catch (Exception e) {
                        e.printStackTrace();
                    }

                    if (images_access){
                        new_image = new SaveImages();
                        new_image.execute(processedDestImage, LEARN_IMAGE);
                    }
                    /*lft_speed = 100; //Right turn
                    rgt_speed = 10; //Right turn*/
                    initialise = true; //Re-call the go command to trigger the speed change
                    avoid = true; //Flag so we know to ignore this code snippet when avoiding
                    dual_accumulator = 0;
                    left_accumulator = 0;
                    right_accumulator = 0;
                    t_move_start = current_time; //Time the saccade started

                } else if (avoid && ((current_time - t_move_start) >= 500)) {
                    //Make adjustments in half second intervals
                    lft_speed = 15; //Back to default
                    rgt_speed = 14; //Back to default
                    avoid = false; //No longer avoiding an obstacle
                    initialise = true; //Need to reset robot speeds
                }


                if (images_access){
                    new_image = new SaveImages();
                    new_image.execute(processedDestImage, LEARN_IMAGE);
                } //Just throwing the code in to learn as many images as possible.

                loop_count++;
                current_time = (int) SystemClock.elapsedRealtime() - start_time; //Update current time
                t_delta = (int) SystemClock.elapsedRealtime() - t_interval_start; //Update accumulation interval

            }

            try {
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        debugTextView.setText(String.format(
                                "End of learning run. Replace the robot at the start of the course " +
                                        "to allow recapitulation."
                        ));
                    }
                });

            } catch (Exception e) {
                e.printStackTrace();
            }

            Command.go(new double[]{0, 0}); //Stop

            try {
                sleep(1000);
            } catch (Exception e) {
                e.printStackTrace();
            }

            if (real) {
                //For real values, want to do multiple memory runs to see how well the model
                //learns over time
                memory_trip = new Thread(navMemory); //Make sure using the real valued nav.
                for ( int run_count = 0; run_count < 5; ++run_count ) {
                    try {
                        sleep(30000); //Wait for 30 Seconds to allow the robot to be replaced at the start of the course
                    } catch (Exception e) {
                        e.printStackTrace();
                    }

                    //Start the attempted reconstruction of the route
                    memory_trip.start();
                    try {
                        memory_trip.join(); //Wait for the thread to finish before next iteration
                    } catch ( Exception e ){ e.printStackTrace(); }
                }
            }else{
                /*try {
                    sleep(30000); //Wait for 30 Seconds to allow the robot to be replaced at the start of the course
                }catch(Exception e){
                    e.printStackTrace();
                }

                //Start the attempted reconstruction of the route
                memory_trip = new Thread(navMemory);
                memory_trip.start();*/



                for ( int run_count = 0; run_count < 5; ++run_count ) {
                    try { Command.turnAround(170); } catch ( Exception e ){ e.printStackTrace(); }
                    try {
                        sleep(5000); //Wait for 30 Seconds to allow the robot to be replaced at the start of the course
                    } catch (Exception e) {
                        e.printStackTrace();
                    }

                    memory_trip = new Thread(navMemory); //Make sure using the real valued nav.
                    //Start the attempted reconstruction of the route
                    memory_trip.start();

                    try {
                        memory_trip.join(); //Wait for the thread to finish before next iteration
                    } catch ( Exception e ){ e.printStackTrace(); }
                }

            }
        }
    };

    //
    // Binary mushroom body memory thread.
    //
    Runnable navMemory = new Runnable(){
        @Override
        public void run() {
            //Assume robot is placed back at the start of the run.
            //Scanning routine?
            //Scan, check familiarity, go in direction of global minima.
            //Using scanning as Klinokinsesis is impractical with this robot in a dense environment

            //This is a modified scanning algorithm from startScanning()
            //Here we rotate the image in the azimuth instead of rotating the robot
            //The image is checked at a rotation of -15 pixels through to 15 pixels (31 total checks)
            //Then the index of greatest familiarity is chosen as per usual
            //We compute the number of notches (pixels) we will need to turn away from our
            //current heading, then multiply by four to get the angle we need passed to turnAround;

            int min_index;
            int start_t = (int) SystemClock.elapsedRealtime();
            int interval_t = (int) SystemClock.elapsedRealtime() - start_t;
            int end_t = 60000; //Allow a lot of extra time for scanning
            int scan_count = 0;
            String task_code = "NAVM";

            while (interval_t < end_t) {
                //Scanning algorithm
                interval_t = (int) SystemClock.elapsedRealtime() - start_t;

                Boolean image_not_accessed;

                String unfamiliarity_distribution = "";

                for (int i = 0; i < 17; i++) {
                    image_not_accessed = true;
                    while (image_not_accessed){
                        if (images_access){
                            Mat matImage = processedDestImage;
                            //Since there is no possibility to send a 2D array or Mat file the image is transformed to a 1D int array
                            //byte[] imageArray_tmp = new byte[matImage.height() * matImage.width()];
                            //matImage.get(0, 0, imageArray_tmp);
                            int rotation = 2 * (i - 8);
                            Log.e("NM", "Image rotation: " + rotation);

                            byte[] imageArray_tmp = Util.rotateMatInAzimuth(rotation, matImage); //Commented to check correctness -
                            int[] imageArray = new int[imageArray_tmp.length];
                            for (int n = 0; n < imageArray_tmp.length; n++) {
                                imageArray[n] = (int) imageArray_tmp[n] & 0xFF;
                            }
                            familiarity_array[i] = mushroom_body.calculateFamiliarity(imageArray);
                            unfamiliarity_distribution += familiarity_array[i] + ",";
                            image_not_accessed = false;
                        }
                    }
                }

                ++scan_count; //Scan complete, increment
                String info_str = "SCN" + scan_count;
                String output = "-t \"Unfamiliarity Distribution for Scan " + scan_count + "\" {" + unfamiliarity_distribution + "}";

                StatFileUtils.write(task_code, info_str, output);

                LogToFileUtils.write(unfamiliarity_distribution+'\n');

                min_index = Util.getMinIndex(familiarity_array);
                output = "Chosen index: " + min_index;
                StatFileUtils.write(task_code, info_str, output);

                LogToFileUtils.write("Seleted index: " + min_index + "\n");
                int notches = 0;
                if (min_index <= 8 ){ //left turn
                    notches = (8 - min_index);
                } else { //right turn
                    notches = -(min_index - 8);
                }

                double degrees = notches * 8;
                output = "Rotation: " + degrees;
                StatFileUtils.write(task_code, info_str, output);
                try {
                    Command.turnAround(degrees);
                } catch (Exception e) {
                    e.printStackTrace();
                }

                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                //We will instead move for a set time, instead of a distance 2 seconds

                Command.go( new double[]{ 15, 14 } );
                try { sleep(1000); } catch ( Exception e ){ e.printStackTrace(); } //Wait for the command to start

                //Time tracking:
                int t_start = (int) SystemClock.elapsedRealtime();
                int t_delta = (int) SystemClock.elapsedRealtime() - t_start;
                int t_interval = 1000; //2 second interval

                //Accumulators for left and right flow
                int dual_accumulator = 0; //Add both values and see if there's significant bias
                int left_accumulator = 0; //Add only left values
                int right_accumulator = 0; //Add only right values
                int accumulation_threshold = 5000; //Threshold for a value to be accumulated (ignore all others)
                int reaction_threshold = 10000; //Value to be met for a reaction to be triggered. (need at most four readings)

                int loop_count = 0;

                while ( t_delta < t_interval ) {
                    //Perform CA accumulation checks
                    try { sleep(600); } catch ( Exception e ){ e.printStackTrace(); }
                    t_delta = (int) SystemClock.elapsedRealtime() - t_start;
                    //Print out debug information, catch exceptions
                    ca_flow_diff = leftCAFlow - rightCAFlow; //If positive, left detection, else right detection

                    //Flow accumulators
                    if ( ca_flow_diff >= accumulation_threshold ){ left_accumulator = left_accumulator + (int) ca_flow_diff; }
                    else if ( ca_flow_diff <= -accumulation_threshold ){ right_accumulator = right_accumulator + Math.abs((int) ca_flow_diff); }

                    if ( Math.abs(ca_flow_diff) >= accumulation_threshold ){ dual_accumulator = dual_accumulator + (int) ca_flow_diff; }

                    //If accumulation time limit reached, reset timer and accumulators
                    if ( loop_count >= 2){
                        left_accumulator = 0;
                        right_accumulator = 0;
                        dual_accumulator = 0;
                        loop_count = 0; //Reset loop counter
                    }

                    if ( (left_accumulator >= reaction_threshold) ||
                            (right_accumulator >= reaction_threshold) || t_delta >=t_interval ) {
                        Command.go(new double[]{0, 0}); //Halt the robot
                        try { sleep(1000); } catch ( Exception e ){ e.printStackTrace(); } //Wait
                        break; //Break the loop (trigger an early scan)
                    }

                    loop_count++;

                }

                try {
                    sleep(2000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

            try {
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        debugTextView.setText(String.format(
                                "End of visual memory run."
                        ));
                    }
                });

            } catch (Exception e) {
                e.printStackTrace();
            }

            int num_images = mushroom_body.imagesLearned();
            StatFileUtils.write("WN", "IL", "Number of images learned for this run: " + num_images);
        }
    };


    //
    // Runnable test thread
    //
    Runnable testThread = new Runnable() {
        //Convert resource into test thread for basic algorithms
        @Override
        public void run() {
            String tag = "OFTST";
            String output = "";
            Mat ident = Mat.eye(3, 3, CvType.CV_8UC1);
            output = "3x3 Identity: \n" + ident.dump();

            Log.e(tag, output);
            Util.rotateMatInAzimuth(1, ident);
            output = "3x3 rotated: \n" + ident.dump();
            Log.e(tag, output);

            Util.rotateMatInAzimuth(1, ident);
            output = "3x3 rotated: \n" + ident.dump();
            Log.e(tag, output);

            Util.rotateMatInAzimuth(1, ident);
            output = "3x3 rotated: \n" + ident.dump();
            Log.e(tag, output);

            Util.rotateMatInAzimuth(-1, ident);
            output = "3x3 rotated: \n" + ident.dump();
            Log.e(tag, output);

            Util.rotateMatInAzimuth(-1, ident);
            output = "3x3 rotated: \n" + ident.dump();
            Log.e(tag, output);

            Util.rotateMatInAzimuth(-1, ident);
            output = "3x3 rotated: \n" + ident.dump();
            Log.e(tag, output);
        }
    };
}
