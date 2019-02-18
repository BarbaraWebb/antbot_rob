package insectsrobotics.imagemaipulations;

import android.os.SystemClock;
import android.util.Log;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.nio.charset.MalformedInputException;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import static org.opencv.imgproc.Imgproc.GaussianBlur;
import static org.opencv.imgproc.Imgproc.INTER_LANCZOS4;
import static org.opencv.imgproc.Imgproc.INTER_LINEAR;
import static org.opencv.imgproc.Imgproc.goodFeaturesToTrack;
import static org.opencv.imgproc.Imgproc.remap;
import static org.opencv.imgproc.Imgproc.resize;
import static org.opencv.video.Video.calcOpticalFlowFarneback;
import static org.opencv.video.Video.calcOpticalFlowPyrLK;

//
// This class will contain all utilities associated with visual pre-processing, including the
// runnable code which maintains the processing list. This works weirdly by setting a bunch of
// global references. It's messy, but it avoided rebuilding half the robot again. Uses the same
// basic idea as the VideoRecorder.
//
// There are a massive amount of variables that are kept in MainActivity that should be in here
// instead; this was due to time constraints.
//
public class VisionUtil {
    //
    // Constructor
    //
    private VisionUtil(){}

    private static Runnable processFrameBuffer = new Runnable(){
        @Override
        public void run(){
            while(true){
                if (!frameBuffer.isEmpty()){
                    Mat displayFrame = processFrame(frameBuffer.removeFirst());
                    framesForDisplay.add(displayFrame);
                } else {
                    app.images_access = false;
                }
            }
        }
    };

    private static LinkedList<Mat> frameBuffer = new LinkedList<Mat>();
    private static LinkedList<Mat> framesForDisplay = new LinkedList<Mat>();
    private static Thread visionThread = new Thread(processFrameBuffer);
    private static float frameRate = 0;
    private static MainActivity app;
    private static int sourceResolution = 1;
    private static int resolution = 4;
    private static int theta_new = 40;
    private static float currentFrameTime;
    private static float prevFrameTime;
    private static boolean calibrate = false;
    private static Mat BlueChannel;
    private static double R1;
    private static double R2;
    private static double unwrapHeight;
    private static double unwrapWidth;
    private static double pixel[];
    private static MatOfByte Status;
    private static MatOfFloat Err;

    public static Mat displayedImage;
    public static boolean displayAvailable = false;
    //
    // Main; run for every frame.
    //
    private static Mat processFrame(Mat rgba){
        initGlobalMats();

        app.images_access = false; // Image access lock

        Mat tempRotate = Mat.zeros(app.processedSourceImage.size(), app.processedSourceImage.type());

        //We have frame rate information computed here
        prevFrameTime = currentFrameTime;
        currentFrameTime = (float) SystemClock.elapsedRealtime();
        if ((currentFrameTime - prevFrameTime) / 1000 > 0){
            frameRate = 1 / ((currentFrameTime-prevFrameTime) / 1000);
        } else {
            frameRate = 0;
        }

        //
        // Calibration:
        // If the camera mount position needs re-calibrated then set the variable "calibrate" to
        // true up above. This will allow the calibration to be run on the first frame from the
        // camera. Run some thread with the robot connected to the PC and filter Logcat using the
        // tag CAL_FUN. Calibrate should auto-detect the position of the ring and give you the
        // centre coordinates. Check the image on the robot is correct. Once you have these, set
        // the values for centreX and centreY accordingly. They need hard-coded so we don't have
        // to try and detect them every time.
        //
        if (calibrate) {
            calibrate = false;
            calibrate(rgba);
        }

        BlueChannel = new Mat(rgba.rows(), rgba.cols(), CvType.CV_8UC1);    //Mat for later image processing

        Mat unwrappedImg =  Mat.zeros((int) unwrapHeight, (int) unwrapWidth, CvType.CV_8UC1);

        double scale = rgba.width() / (360 / resolution);
        double rgbaHeight = rgba.height();

        displayedImage = Mat.zeros((int) (rgbaHeight / scale), 360 / resolution, CvType.CV_8UC1);

        //Extract blue channel from RGBA image and store in BlueChannel - RM
        app.rgbaList.add(rgba);                                                 //Needed for channel extraction from rgba image
        app.BlueChannelList.add(BlueChannel);                                   //Needed for channel extraction from rgba image
        Core.mixChannels(app.rgbaList, app.BlueChannelList, app.from_to);       //Extract only the blue channel from rgba

        //Unwrap circular image - RM
        remap(BlueChannel, unwrappedImg, app.imageMapX, app.imageMapY, INTER_LINEAR);

        //Resize into temp for image rotation - RM
        resize(unwrappedImg, tempRotate, app.processedSourceImage.size(), 2.81, 2.81, INTER_LANCZOS4);

        int counter1 = 0;
        int counter3;
        int colPosition;

        //Image rotation? - RM
        for (int phi = 0; phi < 360; phi = phi + sourceResolution) {
            counter3 = 0;
            for (int theta_tmp = 0; theta_tmp < theta_new; theta_tmp = theta_tmp + sourceResolution) {
                colPosition = counter1 + 180;
                if(colPosition >= 360){
                    colPosition = colPosition - 360;
                }
                pixel = tempRotate.get(counter3, colPosition);
                app.processedSourceImage.put(counter3, counter1, pixel);
                counter3++;
            }
            counter1++;
        }

        //Rotated image now in app.processedSourceImage - RM

        /**
         * If the output resolution is different from 1x1° we have to down-sample the image.
         * openCVs RegionOfInterest method is perfect for this cause.
         *
         */

        //Down-sampling, will always happen as res is hard-coded to 4 - RM
        //Will down-sampleapp.processedSourceImage and output it toapp.processedDestImage - RM
        if (resolution != 1) {
            int destAzimuthCounter = 0;
            for (int azimuth = 0; azimuth <app.processedSourceImage.cols(); azimuth = azimuth + resolution) {
                int destElevationCounter = 0;
                for (int elevation = 0; elevation <app.processedSourceImage.rows(); elevation = elevation + resolution) {
                    Rect roi;
                    //New Rectangle with the target resolution, later the ROI of the frame.
                    roi = new Rect(azimuth, elevation, resolution, resolution);
                    if (app.processedSourceImage.cols() - azimuth < resolution) {
                        roi = new Rect(azimuth, elevation,app.processedSourceImage.cols() - azimuth, resolution);
                    }
                    if (app.processedSourceImage.rows() - elevation < resolution) {
                        roi = new Rect(azimuth, elevation, resolution,app.processedSourceImage.rows() - elevation);
                    }
                    //Getting the pixels of the Region of Interest and averaging the values.
                    Mat ROI =app.processedSourceImage.submat(roi);
                    int ROIMean = (int) Core.mean(ROI).val[0];
                   app.processedDestImage.put(destElevationCounter, destAzimuthCounter, ROIMean);
                    destElevationCounter++;

                }
                destAzimuthCounter++;
            }

        } else {
            /**
             * If the output resolution is the same as the source resolution, we can use the same Image
             */
            app.processedDestImage =app.processedSourceImage;
        }

        //app.processedDestImage is now the down-sampled 90x10, blue channel image needed - RM

        //Some image pre-processing - RM
        Imgproc.equalizeHist(app.processedDestImage,app.processedDestImage);
        GaussianBlur(app.processedDestImage,app.processedDestImage, new Size(3, 3), 0, 0);

        app.images_access = true;

        //Image shifting - RM
        //shift image so that left side is left and right side is right
        app.processedDestImage.colRange(0, 68).copyTo(app.current_image.colRange(22, 90));
        app.processedDestImage.colRange(68, 90).copyTo(app.current_image.colRange(0, 22));

        // Left and Right flow images created here - RM
        // takes 360 image with center at 45 deg for left flow
        app.current_image.colRange(12, 90).copyTo(app.rightCXFlowImage.colRange(0, 78));
        app.current_image.colRange(0, 12).copyTo(app.rightCXFlowImage.colRange(78, 90));

        // takes 360 image with center at -45 deg for right flow
        app.current_image.colRange(0, 78).copyTo(app.leftCXFlowImage.colRange(12, 90));
        app.current_image.colRange(78, 90).copyTo(app.leftCXFlowImage.colRange(0, 12));

        // compute optic flow and charge the global variables with the speed values

        // If app.recording is enabled, hand the current frame to the app.recorder
        if (app.recording){
            app.recorder.recordFrame(app.current_image);
        }

        Log.i("_REC_", "Frame Rate : " + frameRate);

        /*
        computeSparseOpticFlow();
        getObstaclesFromSparseFlow();
        getSpeedsFromSparseFlow();
*/

        //computeDenseOpticFlow();
        //filterCollisionAvoidance(); //Collision avoidance using a flow filter and dense optic flow
        //getSpeedsFromDenseFlow();
        app.speed = (app.leftCXFlow + app.rightCXFlow) / 2;

        displayAvailable = false;

        //Here we put the processed image back into a Mat with the proportions of the output image
        app.current_image.copyTo(
                displayedImage.rowRange(0, app.current_image.rows())
                        .colRange(0, displayedImage.cols())
        );

        app.debugFlowImage.copyTo(
                displayedImage.rowRange(displayedImage.rows() - app.debugFlowImage.rows(), displayedImage.rows())
                        .colRange(0, displayedImage.cols())
        );

        //Resizing of the Image to fit the size of the JavaImageView
        Size size = new Size(rgba.width(), rgba.height());
        resize(displayedImage, displayedImage, size);

        return displayedImage;
    }

    //
    // Utility
    //

    //
    // Auto-detect the 360deg mirror ring in the lens attachment; then build the map based
    // on this position.
    //
    private static void calibrate(Mat rgb_image){
        Util.saveImageToFolder(rgb_image, "image_from__calibrate", "cal");
        Mat hsv = new Mat(rgb_image.rows(), rgb_image.cols(), CvType.CV_8UC1);
        List<Mat> hsv_split = new ArrayList<Mat>();

        Mat blurred = new Mat(rgb_image.rows(), rgb_image.cols(), rgb_image.type());
        Mat circles = new Mat();

        Log.i("CAL_FUN", "Calibrating");
        Imgproc.cvtColor(rgb_image, hsv, Imgproc.COLOR_BGR2HSV);
        Log.i("CAL_FUN", "Extracting saturation");

        //
        // Extract the saturation channel from the HSV image
        //
        Core.split(hsv, hsv_split);

        // Blur the saturation
        // The blurring factor (the Size) can be modified to get a more blurred image, which
        // will be quicker for the Hough transform; however, if you lose too much detail you
        // can no longer detect circles. Size(10,10) as well as param2 == 300 known to work
        // on the brightly lit football pitch; this really would be good if it were more consistent
        Imgproc.blur(hsv_split.get(1), blurred, new Size(10,10));
        Util.saveImageToFolder(blurred, "blurred", "");
        Log.i("CAL_FUN", "Hough transform . . .");
        Imgproc.HoughCircles(
                blurred,
                circles,
                Imgproc.CV_HOUGH_GRADIENT,
                1,
                20,
                30,
                300,
                0,
                0);

        // With these parameters on the blurred image, HoughCircles should detect one circle
        // with its centre at the centre of the camera attachment.
        Log.i("CAL_FUN", "# Circles : " + circles.cols());
        for (int i = 0; i < circles.cols(); i++){
            double[] circle = circles.get(0, i);
            app.centreX = circle[0];
            app.centreY = circle[1];
        }

        Log.i("CAL_FUN", "Detected centre : (" + app.centreX + ", " + app.centreY + ")");
        // Build the unwrap map based on the computed centre (will default to the manually set
        // values if no centre is detected).
        unwrapMap();
    }

    //
    // Build the image unwrap map
    //
    private static void unwrapMap(){
        double r;
        double theta;
        int xS;
        int yS;
        int l = 0;
        Mat temp= Mat.zeros(2,2,CvType.CV_32FC1);

        //
        // Build the Affine Matrix
        //
        for (int x = 0;x < app.affineMat.rows(); x++){
            for (int y = 0;y < app.affineMat.cols(); y++){
                app.affineMat.put(x, y, app.affine[l]);
                l++;
            }
        }

        for (int y = 0; y < (int) unwrapHeight; y++){
            for (int x = 0; x < (int) unwrapWidth; x++){
                r = ((double) y / unwrapHeight) * (app.R2 - app.R1) + app.R1;
                theta = ((double) x / unwrapWidth) * 2 * Math.PI;

                //
                // Polar coordinates of the current pixel
                //
                xS = (int) (app.centreX + r*Math.sin(theta));
                yS = (int) (app.centreY + r*Math.cos(theta));

                temp.put(0, 0, xS);
                temp.put(1, 0, yS);
                Core.gemm(app.affineMat.inv(), temp,1,new Mat(),0, temp,0);

                xS = (int)(temp.get(0,0)[0]);
                yS = (int)temp.get(1,0)[0];

                app.imageMapX.put(y, x, xS);
                app.imageMapY.put(y, x, yS);
            }
        }
    }

    //
    // Initialise the matrices which are stored in MainActivity
    //
    private static void initGlobalMats(){

        app.rgbaList = new ArrayList<>();
        app.BlueChannelList = new ArrayList<>();
        app.from_to = new MatOfInt(2, 0);
        app.rgba = new  Mat();

        // Init left and right CX images
        app.currentRightCXImage = Mat.zeros(10, 90, CvType.CV_8UC1);
        app.currentLeftCXImage = Mat.zeros(10, 90, CvType.CV_8UC1);

        // Init left and right CX flow matrices for speed tracking
        app.leftCXFlowImage = Mat.zeros(10, 90, CvType.CV_8UC1);
        app.rightCXFlowImage = Mat.zeros(10, 90, CvType.CV_8UC1);

        // Init current and debug matrices
        app.current_image = Mat.zeros(10, 90, CvType.CV_8UC1);
        app.debugFlowImage = Mat.zeros(10, 90, CvType.CV_8UC1);

        // Init processing matrices
        app.processedSourceImage = Mat.zeros(theta_new / sourceResolution, 360 / sourceResolution, CvType.CV_8UC1);
        app.processedDestImage = Mat.zeros(theta_new / resolution, 360 / resolution, app.processedSourceImage.type());
    }

    //
    // Compute Dense optical flow using the Farneback method
    //
    private static void computeDenseOpticFlow() {
        // first time around we initialize all
        if (app.previous_image == null) {

            app.previous_image = Mat.zeros(10, 90, CvType.CV_8UC1);
            app.currentPointsToTrack = new MatOfPoint2f();
            Status = new MatOfByte();
            Err = new MatOfFloat();

            app.current_image.copyTo(app.previous_image);
        }

        calcOpticalFlowFarneback(
                app.previous_image,
                app.current_image,
                app.currentPointsToTrack,
                0.5,
                1,
                5,
                3,
                3,
                1.1,
                0
        );
        app.tracked = true;
    }

    //
    // Compute Sparse optical flow using the Lucas-Kanade method
    //
    private static void computeSparseOpticFlow() {
        // first time around we initialize all
        // Looking at repurposing this method for optical flow obstacle avoidance - RM
        if (app.previous_image == null) {

            app.previous_image = Mat.zeros(10, 90, CvType.CV_8UC1);
            app.initialImage = new MatOfPoint();
            app.currentPointsToTrack = new MatOfPoint2f();
            app.prevPointsToTrack = new MatOfPoint2f();
            Status = new MatOfByte();
            Err = new MatOfFloat();

            app.current_image.copyTo(app.previous_image);
            goodFeaturesToTrack(app.current_image, app.initialImage, 100, 0.5, 3);
            app.initialImage.convertTo(app.prevPointsToTrack, CvType.CV_32FC2);
        }

        //What? Only the left flow is computed??? - RM
        //Seemingly only using the central image so regular flow is computed -- RM
        // now we can compute the flow
        // left flow

        //Seems to be an embellished if ( !(prevPointsToTrack == null) ) {}
        if (app.prevPointsToTrack.cols() > 0 && app.prevPointsToTrack.rows() > 0) {
            calcOpticalFlowPyrLK(
                    app.previous_image,
                    app.current_image,
                    app.prevPointsToTrack,
                    app.currentPointsToTrack, // variable holding the new point positions
                    Status,
                    Err,
                    new Size(9, 9),
                    2
            );
            app.tracked = true;
        } else {
            app.tracked = false;
        }
    }

    //
    // Compute collision avoidance information using optical flow
    //
    private static void filterCollisionAvoidance() {
        int delta = (int) SystemClock.elapsedRealtime() - app.global_current_time; //Change in time since last read
        app.global_current_time = (int) SystemClock.elapsedRealtime() - app.global_start_time; //Time since start

        Mat left_filter = CX_Holonomic.get_preferred_flow(90, Math.toRadians(0), true); //Left flow filter
        Mat right_filter = CX_Holonomic.get_preferred_flow(90, Math.toRadians(0), false); //Right flow filter

        //Filters are functionally identical, see Lucas' dissertation for the method used

        double[] current_left_flow_vector;
        double[] current_right_flow_vector;
        double[] previous_left_flow_vector;
        double[] previous_right_flow_vector;

        Mat left_filter_vector;
        Mat right_filter_vector;

        Mat flow_vector = new Mat(1, 3, CvType.CV_32FC1);

        float left_flow_sum = 0;
        float right_flow_sum = 0;

        int offset = 4;

        for ( int y = 0; y < app.currentPointsToTrack.rows(); y++ ){
            for ( int x = 0; x < app.currentPointsToTrack.cols(); x++ ){
                //These filters are functionally identical, see Luca's dissertation for the method used to compute the filter
                //Compute left flow vector
                //+-4 is the left/right offset for the flow frame; e.g. -4 centres the frame at -16deg.
                //The narrower this is, the closer we are to considering the original flow filter.
                //So this may effectively act as our rather than the raw pixel values.
                previous_left_flow_vector = new double[]{Util.mod(x + offset, 90), y}; //((x+12), y)
                current_left_flow_vector = new double[]{Util.mod((int) app.currentPointsToTrack.get(y, x)[0] + x + offset, 90),
                        Util.mod((int) app.currentPointsToTrack.get(y, x)[1] + y, 10)}; //Flow info returned by farneback

                if ( (x >= 40) || (x < 50) ) {
                    left_filter_vector = left_filter.row((int) previous_left_flow_vector[0]);

                    //Create 1x3 flow vector (the current flow vector)
                    flow_vector.put(0, 0, current_left_flow_vector[0]);
                    flow_vector.put(0, 1, current_left_flow_vector[1]);
                    flow_vector.put(0, 2, 0);

                    left_flow_sum += left_filter_vector.dot(flow_vector); //Filter and sum
                }
                //Compute right flow vector
                previous_right_flow_vector = new double[] { Util.mod(x - offset, 90), y };
                current_right_flow_vector = new double[] { Util.mod((int) app.currentPointsToTrack.get(y,x)[0] + x - offset, 90),
                        Util.mod((int) app.currentPointsToTrack.get(y,x)[1] + y, 10) };

                if ( (x >= 40) || (x < 50) ) {
                    right_filter_vector = right_filter.row((int) previous_right_flow_vector[0]); //Vector for x + 12 mod 90

                    //Create 1x3 flow vector (the current flow vector)
                    flow_vector.put(0, 0, current_right_flow_vector[0]);
                    flow_vector.put(0, 1, current_right_flow_vector[1]);
                    flow_vector.put(0, 2, 0);

                    right_flow_sum += right_filter_vector.dot(flow_vector); //Filter and sum
                }
            }
        }



        //Again due to image shifting, left is right and right is left.
        app.rightCAFlow =  1000 * left_flow_sum;// / (delta /*900*/);
        app.leftCAFlow = 1000 * right_flow_sum; // / (delta /*900*/);

    }
    
    //
    // Public methods
    //

    //
    // Set the visual processing thread to a maximum priority thread, and start it running; this
    // acts as the constructor.
    //
    public static void initialise(MainActivity ma){
        app = ma;
        R1 = app.R1;
        R2 = app.R2;
        unwrapWidth = app.unwrapWidth;
        unwrapHeight = app.unwrapHeight;

        visionThread.setPriority(10);
        visionThread.start();
    }

    //
    // Add a frame to the buffer for processing.
    //
    public static void queueFrame(Mat frame){
        frameBuffer.add(frame);
    }

    public static Mat getFrameForDisplay(){
        if (framesForDisplay.isEmpty()){
            Mat blank = Mat.zeros(10,90, CvType.CV_8UC1);
            Size size = new Size(1024,768);
            Imgproc.resize(blank, blank, size);
            return blank;
        }

        return framesForDisplay.removeFirst();
    }
}
