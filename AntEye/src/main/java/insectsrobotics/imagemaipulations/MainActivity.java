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

public class MainActivity extends Activity implements CvCameraViewListener2 , BroadcastValues, SensorEventListener{
    protected static final String TAG = "OCVSample::Activity";
    boolean opticCheck = false;
    public Mat processedSourceImage;
    public Mat processedDestImage;
    public Mat displayedImage;
    Button calcErrorBtn;
    int counting = 0;
    Mat imageToDisplay;

    //Receiver and Broadcaster
    Broadcast broadcast;
    Receive receive;

    //Image processing variables
    Mat BlueChannel;
    MatOfInt from_to;
    Mat rgba;
    Mat norm_rgba;
    List<Mat> rgbaList;
    List<Mat> BlueChannelList;
    double[] pixel;
    String myTag ="Canny";

    // ----- CX variables ------
    int startTime;
    float frame_rate_cx = 0;
    protected double currentDegree = 0.;
    protected double CXnewHeading;
    protected double CXtheta;
    protected double CXmotor = 0;
    protected double CXmotorChange = 0.5;
    double ANT_SPEED = 1.;
    private SensorManager mSensorManager;
    private Sensor mSensor;
    protected int CURRiteration;

    Mat leftCXFlowImage = null;
    Mat rightCXFlowImage = null;
    Mat currentRightCXImage;
    Mat currentLeftCXImage;
    Mat current_image;
    Mat previous_image;
    MatOfPoint initialImage;
    MatOfPoint2f prevPointsToTrack;
    MatOfPoint2f currentPointsToTrack;
    MatOfByte Status;
    MatOfFloat Err;
    Mat debugFlowImage;
    boolean tracked = false;
    float prevFrameTime = 0;
    float currentFrameTime;
    float rightCXFlow = 0;
    float leftCXFlow = 0;
    double imageCorrection;
    int cameraViewStartedWidth = 0;
    int cameraViewStartedHeight = 0;

    //Calibration resulting variables
    Bundle mBundle;
    double[] theta_new = new double[40];

    //Unwrapping variables
    int sourceResolution = 1;
    int resolution = 4;

    //Optical flow variables
    Mat second_frame;

    //Module Selection from StartScreen
    String visualModule;
    String pathIntegratorModule;
    String combinerModule;
    String opticalFlowModule;
    String visualNavigationModule;

    //Layout Views
    TextView serialConnectionTextView;
    TextView debugTextView;
    ProgressBar serialProgressBar;
    ProgressBar serverProgressBar;
    ImageView serialCheckImageView;
    ImageView serverCheckImageView;
    AlertDialog.Builder builder;

    //DEBUG variables to save and load images from txt files
    int numberOfImages = 0;
    File sdCard = Environment.getExternalStorageDirectory();
    File dir = new File(sdCard.getAbsolutePath() + "/");
    File file = new File(dir, "text.txt");
    File log_file = new File( dir, "log_file.txt");

    //*** Camera calibration data
    double R1 = 104.7; //radius of the inner circle
    double R2 = 217.2; //radius of the outer circle
    double centreX = 527; // X coordinate of the image centre
    double centreY = 406; // Y coordiate of the image centre
    double[] affine = new double[]{1.000329,0.000171,-0.000339,1.0}; // Affine parameters c, d, e and 1
    Mat affineMat;

    //** Unwrapped image parameters
    double unwrapHeight = 0;
    double unwrapWidth = 0;
    Mat imageMapX;
    Mat imageMapY;
    double scale =1;
    Mat mGray2;
    boolean sec_check=false;

    //Visual Homing
    public Mat snapShot;
    public Mat fullSnapShot;
    public Mat fullImageToDisplay;
    public Mat temporaryImageToCompare;
    List<Mat> rotatedImageDB;
    boolean savedList=false;
    boolean stopThread=false;
    Thread startHoming;

    //PI
    Thread obstacleAvoid;
    int integratorRunTime=35000; //outbound time (milliseconds)
    int selectedModule=0;

    //Combiner
    int MethodChosen;

    // Choose from one of the following models for route following
    WillshawModule model = new WillshawModule();

    // The combined CX and MB model
    CX_MB cxmb = new CX_MB();

    // Stored memory to retrieve the CPU memory again
    SimpleMatrix stored_memory = new SimpleMatrix(CX.n_cpu4, 1);

    AsyncTask<Object, Integer, Boolean> new_image = null;
    AsyncTask<Object, Integer, Boolean> start_home = null;
    Boolean images_access = false;
    double[] familiarity_array = new double[17]; //Rob
    double[] familarity_array = new double[11]; //Zhaoyu

    //Optical flow obstacle detection - RM
    //Will reuse some visual variables

    //Single thread will be used for both runnables: Detection and avoidance
    Thread opticalFlowThread; //Bot will detect obstacle and try to navigate it

    int threshold_ttc = 0;
    float speed = Math.abs( (leftCXFlow + rightCXFlow) / 2);

    //Left and right collision avoidances which may be thresholded. If one side exceeds a threshold
    //Saccade in opposite direction
    float leftCAFlow = 0;
    float rightCAFlow = 0;
    float ca_flow_diff = 0;

    int global_start_time = (int) SystemClock.elapsedRealtime();
    int global_current_time = (int) SystemClock.elapsedRealtime() - global_start_time;

    //Visual Navigation Through Dense Environments : VN_Vars
    Thread learning_trip; //Thread for the outbound learning trip
    Thread memory_trip; //Thread for reproducing the route learned in the learning_trip

    boolean real = false; //Boolean to flag which network we use
    WillshawModule mushroom_body;
    RealWillshawModule real_mushroom_body;

    //Initiate all ServiceConnections for all Background Services
    ServiceConnection visualNavigationServiceConnection = new ServiceConnection() {
        @Override
        public void onServiceConnected(ComponentName name, IBinder service) {
            Log.i(myTag,"VisualNavigationService Started");
        }

        @Override
        public void onServiceDisconnected(ComponentName name) {
        }
    };
    ServiceConnection serialServiceConnection = new ServiceConnection() {
        @Override
        public void onServiceConnected(ComponentName name, IBinder service) {
        }

        @Override
        public void onServiceDisconnected(ComponentName name) {
        }
    };
    ServiceConnection combinerServiceConnection = new ServiceConnection() {
        @Override
        public void onServiceConnected(ComponentName name, IBinder service) {
        }

        @Override
        public void onServiceDisconnected(ComponentName name) {
        }
    };
    ServiceConnection integratorServiceConnection = new ServiceConnection() {
        @Override
        public void onServiceConnected(ComponentName name, IBinder service) {

        }

        @Override
        public void onServiceDisconnected(ComponentName name) {

        }
    };

    //
    // Thread containers. The oldThreads object holds threads pre-2018 project, threads holds 2018+
    //
    OldThreadCode oldThreads;
    ThreadCode threads;

    //The UI Views
    private CameraBridgeViewBase mOpenCvCameraView;

    //Start OpenCV, open CameraBridge
    private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS: {
                    Log.i(TAG, "OpenCV loaded successfully");
                    imageToDisplay=Mat.zeros(10,90,CvType.CV_8UC1);
                    // Set Camera parameters
                    unwrapHeight = scale*(R2-R1);
                    unwrapWidth = scale*2*Math.PI*(R1+R2)/2;
                    imageMapX = Mat.zeros(((int)unwrapHeight),((int) unwrapWidth),CvType.CV_32FC1);
                    imageMapY = Mat.zeros(((int)unwrapHeight),((int) unwrapWidth),CvType.CV_32FC1);
                    affineMat = new Mat(2,2,CvType.CV_32FC1);
                    unwrapMap();

                    mGray2 = Mat.zeros(theta_new.length / sourceResolution, 360 / sourceResolution, CvType.CV_8UC1);
                    second_frame= Mat.zeros(theta_new.length / resolution, 360 / resolution,CvType.CV_8UC1);
                    mOpenCvCameraView.enableView();
                }
                break;
                default: {
                    super.onManagerConnected(status);
                }
                break;
            }
        }
    };


    /**
     * Declare a Receiver and its Listener. The Receiver implements the Listener. On a receive with
     * an intent with the in the IntentFilter declared action, the Listeners onReceive Method is
     * called.
     */
    Receive.ReceiveListener receiveListener = new Receive.ReceiveListener() {
        @Override
        public void onNewMessageFromSerialConnectionApp(Intent intent, String action) {
            if (action.equals(SERIAL_CONNECTION_ESTABLISHED)){
                if (intent.getBooleanExtra("MainData", false)){
                    serialProgressBar.setVisibility(View.GONE);
                    serialCheckImageView.setVisibility(View.VISIBLE);
                } else {
                    serialProgressBar.setVisibility(View.VISIBLE);
                    serialCheckImageView.setVisibility(View.GONE);
                }
            } else if (action.equals(SERVER_CONNECTION_ESTABLISHED)){
                if (intent.getBooleanExtra("MainData", false)) {
                    serverProgressBar.setVisibility(View.GONE);
                    serverCheckImageView.setVisibility(View.VISIBLE);
                } else {
                    serverProgressBar.setVisibility(View.VISIBLE);
                    serverCheckImageView.setVisibility(View.GONE);
                }
            }
        }

        @Override
        public void onNewMessageFromVisualNavigationApp(Intent intent, String action) {
            if (action.equals(NUMBER_OF_IMAGES)){
                numberOfImages = intent.getIntExtra("MainData", 0);
            }
        }

        @Override
        public void onNewMessageFromCombinerApp(Intent intent, String action) {
            switch (action){
                case TURN_AE_DATA:
                    try {
                        Command.turnAround(intent.getDoubleExtra("MainData",0));
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    break;
            }
        }

        @Override
        public void onNewMessageFromIntegratorApp(Intent intent, String action) {}

        @Override
        public void onNewMessageFromAntEyeApp(Intent intent, String action) {}
    };

    // Zhaoyu added this to
    // Show the alert dialog after first run.
    protected void showDialog() {
        builder = new AlertDialog.Builder(this);
        builder.setTitle(R.string.put_back_title);
        builder.setMessage(R.string.place_back_food);

        builder.setPositiveButton(R.string.yes, new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialogInterface, int id) {
                start_home = new Startback();
                start_home.execute();
            }
        });

        AlertDialog dialog = builder.create();
        dialog.show();
    }

    //MainActivity Constructor
    public MainActivity() {
    }

    /**
     * Called when the activity is first created.
     */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        Log.i("MainActivity", "Main Activity starts");

        //First Bind all other services from apps to this one
        Intent visualNavigationServiceIntent = new Intent(VISUAL_SERVICE);
        bindService(visualNavigationServiceIntent, visualNavigationServiceConnection, BIND_AUTO_CREATE);

        Intent combinerServiceIntent = new Intent(COMBINER_SERVICE);
        bindService(combinerServiceIntent, combinerServiceConnection, BIND_AUTO_CREATE);

        Intent integratorServiceIntent = new Intent(INTEGRATOR_SERVICE);
        bindService(integratorServiceIntent, integratorServiceConnection, BIND_AUTO_CREATE);

        Intent serialServiceIntent = new Intent(SERIAL_SERVICE);
        bindService(serialServiceIntent, serialServiceConnection, BIND_AUTO_CREATE);


        //Initiate all Receiver and Broadcaster
        broadcast = new Broadcast(this,getResources());
        receive = new Receive(receiveListener);
        Command.setBroadcast(broadcast);

        IntentFilter intentFilter = receive.getIntentFilter();
        registerReceiver(receive, intentFilter);

        //Inflate Layout and initiate the Views
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        setContentView(R.layout.activity_image_manipulations);
        mOpenCvCameraView = (CameraBridgeViewBase) findViewById
                (insectsrobotics.imagemaipulations.Build.R.id.main_activity_surface_view);

        if (mOpenCvCameraView != null)
        {
            Log.e("INFO", "CameraView is not null!");
        }
        else
        {
            Log.e("INFO", "CameraView is null!");
        }


        mOpenCvCameraView.setCvCameraViewListener(this);

        mSensorManager = (SensorManager) getSystemService(this.SENSOR_SERVICE);
        mSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR);

        serialConnectionTextView = (TextView) findViewById(R.id.serialConnectionTextView);
        debugTextView = (TextView) findViewById(R.id.debugTextView);
        serialProgressBar = (ProgressBar) findViewById(R.id.serialProgressBar);
        serialCheckImageView = (ImageView) findViewById(R.id.serialCheckImageView);
        serialCheckImageView.setImageResource(R.drawable.checksymbol);

        //Loads calibration_layout data from passed Intent (Coming from StartScreen) and Advanced Settings
        Intent intent = getIntent();

        //Get Data from Intent
        mBundle = intent.getBundleExtra("Data");

        //Get Module Selection from Bundle
        visualModule = mBundle.getString(VN_MODULE, NO_MODULE);
        pathIntegratorModule = mBundle.getString(PI_MODULE, NO_MODULE);
        combinerModule = mBundle.getString(C_MODULE, NO_MODULE);
        opticalFlowModule = mBundle.getString(OF_MODULE, NO_MODULE);
        visualNavigationModule = mBundle.getString(NAV_MODULE, NO_MODULE);
        selectedModule=mBundle.getInt("selectModule",0);

        // DEBUG LOG to file
        LogToFileUtils.init(this.getApplicationContext());
        StatFileUtils.init(this.getApplicationContext()); //Statistical log file
        StatFileUtils.write("new", "new", "new"); //Add comments to delimit the instances of MainActivity in the stats file

        //threads = new ThreadCode(this);
        //oldThreads = new OldThreadCode(this);

        try {
            FileOutputStream f = new FileOutputStream(file);
            f.write(("Measurements: " + "\n").getBytes());
        } catch (Exception e) {
            e.printStackTrace();
        }

    }

    /**
     * Disables CameraBridge/ view on activity pause to prevent memory leaks
     */
    @Override
    public void onPause() {
        super.onPause();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
        // to stop the listener and save battery
        mSensorManager.unregisterListener(this);
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        float x = event.values[0];
        float y = event.values[1];
        float z = event.values[2];
        float w = event.values[3];

        double ysqr = y * y;
        double a = -2.0f * (ysqr + z* z) + 1.0f;
        double b = +2.0f * (x * y - w * z);
        double yaw = Math.atan2(b, a);
        double angle = Math.round(Math.toDegrees(yaw));

        if(angle < 0){
            angle += 360;
        }

        currentDegree = angle;
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {}

    /**
     * Enables view again on resume of the activity
     */
    @Override
    public void onResume() {
        super.onResume();
        Log.d("onResume", "onResume called");
        imageCorrection = 0;

        if (!OpenCVLoader.initDebug()) {
            Log.d(TAG, "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, this, mLoaderCallback);
        } else {
            Log.d(TAG, "OpenCV library found inside package. Using it!");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }

        if (cameraViewStartedWidth != 0 && cameraViewStartedHeight != 0) {
            onCameraViewStarted(cameraViewStartedWidth, cameraViewStartedHeight);
        }

        // for the system's orientation sensor registered listeners
        mSensorManager.registerListener(this, mSensor, SensorManager.SENSOR_DELAY_NORMAL);
    }

    /**
     * Disables view to prevent memory leaks after the activity was destroyed
     */
    public void onDestroy() {
        super.onDestroy();

        unbindService(visualNavigationServiceConnection);
        unbindService(combinerServiceConnection);
        unbindService(serialServiceConnection);
        unbindService(integratorServiceConnection);
        unregisterReceiver(receive);
        stopThread = true;
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    /**
     * Called after successful initiation of the camera connection, initiates Variables, sets up
     * a connection array between the "Donut-View" and the unwrapped image.
     */
    public void onCameraViewStarted(int width, int height) {
        Log.e(TAG, "onCameraViewStarted called");

        cameraViewStartedWidth = width;
        cameraViewStartedHeight = height;
    }

    public void onCameraViewStopped() {}

    /**
     *  Building a Map to unwrap the image
     */
    private void unwrapMap(){
        double r;
        double theta;
        int xS;
        int yS;
        int l=0;
        Mat temp= Mat.zeros(2,2,CvType.CV_32FC1);

        for(int x = 0; x < affineMat.rows(); x++){
            for(int y = 0; y < affineMat.cols(); y++){
                affineMat.put(x,y,affine[l]);
                l++;
            }
        }

        for(int y = 0; y < (int) unwrapHeight; y++){
            for(int x = 0; x < (int) unwrapWidth; x++){
                r = ((double) y / unwrapHeight) * (R2 - R1) + R1;
                theta = ((double) x / unwrapWidth) * 2 * Math.PI;
                xS = (int) (centreX + r * Math.sin(theta));
                yS = (int) (centreY + r * Math.cos(theta));

                temp.put(0, 0, xS);
                temp.put(1, 0, yS);
                Core.gemm(affineMat.inv(),temp,1,new Mat(),0,temp,0);
                xS=(int)(temp.get(0,0)[0]);
                yS=(int)temp.get(1,0)[0];

                imageMapX.put(y, x, xS);
                imageMapY.put(y, x, yS);
            }
        }
    }

    /**
     * openCv Method with inputFrame from FrontCamera, imageProcessing and output to display.
     * At the same time the endless loop to do more or less all the work.
     *
     * @param inputFrame Frame from the cameraViewListener
     * @return the displayed Image for the Screen. Important: the returned Image has to be the same
     * size as the inputFrame!
     */
    //
    // Much of onCameraFrame could be refactored. However, this will take time and testing to
    // accomplish.
    //
    public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
        //Callback method, every camera frame received will go through here. - RM

        //Initiation of the needed Variables
        rgbaList = new ArrayList<>();
        BlueChannelList = new ArrayList<>();
        from_to = new MatOfInt(2, 0);
        rgba = new Mat();
        norm_rgba = new Mat();

        // initialize some CX variables
        currentRightCXImage = Mat.zeros(10, 90, CvType.CV_8UC1);
        currentLeftCXImage = Mat.zeros(10, 90, CvType.CV_8UC1);

        leftCXFlowImage = Mat.zeros(10, 90, CvType.CV_8UC1);
        rightCXFlowImage = Mat.zeros(10, 90, CvType.CV_8UC1);

        //Centered flow image - For obstacle detection - RM
        //centeredFlowImage = Mat.zeros(10, 90, CvType.CV_8UC1);

        current_image = Mat.zeros(10, 90, CvType.CV_8UC1);
        debugFlowImage = Mat.zeros(10, 90, CvType.CV_8UC1);

        processedSourceImage = Mat.zeros(theta_new.length / sourceResolution, 360 / sourceResolution, CvType.CV_8UC1);

        processedDestImage = Mat.zeros(theta_new.length / resolution, 360 / resolution, processedSourceImage.type());
        images_access = false;
        Mat tempRotate = Mat.zeros(processedSourceImage.size(), processedSourceImage.type());
        fullImageToDisplay=Mat.zeros(processedSourceImage.size(),processedSourceImage.type());

        //We have frame rate information computed here
        prevFrameTime = currentFrameTime;
        currentFrameTime = (float) SystemClock.elapsedRealtime();
        if ((currentFrameTime-prevFrameTime)/1000 > 0){
            frame_rate_cx = 1/((currentFrameTime-prevFrameTime)/1000);
        } else {
            frame_rate_cx = 0;
        }

        //Get input frame in RGBA then copy to BlueChannel Mat - RM
        rgba = inputFrame.rgba();                                           //Input Frame in rgba format
        BlueChannel = new Mat(rgba.rows(), rgba.cols(), CvType.CV_8UC1);    //Mat for later image processing

        Mat unwrappedImg =  Mat.zeros((int) unwrapHeight, (int) unwrapWidth, CvType.CV_8UC1);

        double scale = rgba.width() / (360 / resolution);
        double rgbaHeight = rgba.height();
        displayedImage = Mat.zeros((int) (rgbaHeight / scale), 360 / resolution, CvType.CV_8UC1);

        //Extract blue channel from RGBA image and store in BlueChannel - RM
        rgbaList.add(rgba);                                                 //Needed for channel extraction from rgba image
        BlueChannelList.add(BlueChannel);                                   //Needed for channel extraction from rgba image
        Core.mixChannels(rgbaList, BlueChannelList, from_to);               //Extract only the blue channel from rgba

        //Unwrap circular image - RM
        remap(BlueChannel, unwrappedImg, imageMapX, imageMapY, INTER_LINEAR);

        //Resize into temp for image rotation - RM
        resize(unwrappedImg, tempRotate, processedSourceImage.size(), 2.81, 2.81, INTER_LANCZOS4);

        int counter1 = 0;
        int counter3;
        int colPosition;

        //Image rotation? - RM
        for (int phi = 0; phi < 360; phi = phi + sourceResolution) {
            counter3 = 0;
            for (int theta_tmp = 0; theta_tmp < theta_new.length; theta_tmp = theta_tmp + sourceResolution) {
                colPosition=counter1+180;
                if(colPosition>=360){
                    colPosition=colPosition - 360;
                }
                pixel = tempRotate.get(counter3,colPosition);
                processedSourceImage.put(counter3, counter1, pixel);
                counter3++;
            }
            counter1++;
        }

        /**
         * If the output resolution is different from 1x1° we have to down-sample the image.
         * openCVs RegionOfInterest method is perfect for this cause.
         *
         */

        //Down-sampling, will always happen as res is hard-coded to 4 - RM
        //Will down-sample processedSourceImage and output it to processedDestImage - RM
        if (resolution != 1) {
            int destAzimuthCounter = 0;
            for (int azimuth = 0; azimuth < processedSourceImage.cols(); azimuth = azimuth + resolution) {
                int destElevationCounter = 0;
                for (int elevation = 0; elevation < processedSourceImage.rows(); elevation = elevation + resolution) {
                    Rect roi;
                    //New Rectangle with the target resolution, later the ROI of the frame.
                    roi = new Rect(azimuth, elevation, resolution, resolution);
                    if (processedSourceImage.cols() - azimuth < resolution) {
                        roi = new Rect(azimuth, elevation, processedSourceImage.cols() - azimuth, resolution);
                    }
                    if (processedSourceImage.rows() - elevation < resolution) {
                        roi = new Rect(azimuth, elevation, resolution, processedSourceImage.rows() - elevation);
                    }
                    //Getting the pixels of the Region of Interest and averaging the values.
                    Mat ROI = processedSourceImage.submat(roi);
                    int ROIMean = (int) Core.mean(ROI).val[0];
                    processedDestImage.put(destElevationCounter, destAzimuthCounter, ROIMean);
                    destElevationCounter++;

                }
                destAzimuthCounter++;
            }
        } else {
            /**
             * If the output resolution is the same as the source resolution, we can use the same Image
             */
            processedDestImage = processedSourceImage;
        }

        //processedDestImage is now the down-sampled 90x10, blue channel image needed - RM

        //Some image pre-processing - RM
        Imgproc.equalizeHist(processedDestImage, processedDestImage);
        GaussianBlur(processedDestImage, processedDestImage, new Size(3, 3), 0, 0);

        images_access = true;

        //Image shifting - RM
        //shift image so that left side is left and right side is right
        processedDestImage.colRange(0, 68).copyTo(current_image.colRange(22, 90));
        processedDestImage.colRange(68, 90).copyTo(current_image.colRange(0, 22));


        // Left and Right flow images created here - RM
        // takes 360 image with center at 45 deg for left flow
        current_image.colRange(12, 90).copyTo(rightCXFlowImage.colRange(0, 78));
        current_image.colRange(0, 12).copyTo(rightCXFlowImage.colRange(78, 90));

        // takes 360 image with center at -45 deg for right flow
        current_image.colRange(0, 78).copyTo(leftCXFlowImage.colRange(12, 90));
        current_image.colRange(78, 90).copyTo(leftCXFlowImage.colRange(0, 12));

        // compute optic flow and charge the global variables with the speed values

        // OPTICAL FLOW COMPUTED HERE - RM

/*
        computeSparseOpticFlow();
        getObstaclesFromSparseFlow();
        getSpeedsFromSparseFlow();
*/
        computeDenseOpticFlow();

        filterCollisionAvoidance(); //Collision avoidance using a flow filter and dense optic flow
        getSpeedsFromDenseFlow();
        speed = (leftCXFlow + rightCXFlow) / 2;

        // opticCheck initialised to false, not ever reset. - RM
        // counting initialised to 1, reset after it reaches 20 - RM
        // So this if waits seven frames then runs this code once. - RM

        if( !opticCheck && counting > 7 ){
            temporaryImageToCompare=Mat.zeros(processedDestImage.size(),processedDestImage.type());
            opticCheck = true;
            fullSnapShot = Mat.zeros(processedSourceImage.size(),processedSourceImage.type());
            snapShot = Mat.zeros(processedDestImage.size(),processedDestImage.type());

            processedDestImage.copyTo(snapShot); //Down-sampled copy of the image frame - RM
            processedSourceImage.copyTo(fullSnapShot); // Full res copy of the image frame - M
            rotatedImageDB = new ArrayList<>();

            //Search hook : MENU_SELECTION
            //Choose which module and which setting to run - RM
            if(selectedModule==0){
                // Uncomment to start recular CX (one speed input)
                //obstacleAvoid=new Thread(CXthread);
                // Uncomment to start Holonomic CX (two speed inputs)
                obstacleAvoid=new Thread(oldThreads.CXHolonomicThread);
                obstacleAvoid.start();
            }else if (selectedModule == 1){ //If selected module is for visual navigation
                /*
                switch(visualModule){  //Select type of navigation to be run

                    case RUNDOWN:
                        // Starting all RunDownThreads
                        startHoming=new Thread(startHome);
                        startHoming.start();
                        runDown=new Thread(saveImagesRunDown);
                        runDown.start();
                        break;
                    case TURN_TO_MIN:
                        //Starting all Turn to minimum Threads
                        startHoming=new Thread(startHome);
                        startHoming.start();
                        turnToMinimum=new Thread(saveImagesTurnMinimum);
                        turnToMinimum.start();
                        break;
                    case SYSTEMATIC_SEARCH:
                        //Starting all search threads
                        startSearching=new Thread(startSearch);
                        startSearching.start();
                        search=new Thread(saveImagesRunDown);
                        search.start();
                        break;
                    default:
                        // Starting all RunDownThreads
                        startHoming=new Thread(startHome);
                        startHoming.start();
                        runDown=new Thread(saveImagesRunDown);
                        runDown.start();
                        break;
                }
                */
            } else if (selectedModule == 2) { //Else run the combiner module
                switch(combinerModule){
                    case BACK_WITH_MB:
                        MethodChosen = 0;
                        obstacleAvoid=new Thread(oldThreads.CXthread);
                        obstacleAvoid.start();
                        startHoming = new Thread(oldThreads.startScanning);
                        break;
                    case KLINOKINESIS:
                        MethodChosen = 1;
                        obstacleAvoid=new Thread(oldThreads.CXthread);
                        obstacleAvoid.start();
                        startHoming = new Thread(oldThreads.startKlinokinesis);
                        break;
                    case EIGHT_ENS:
                        MethodChosen = 2;
                        obstacleAvoid = new Thread(oldThreads.CXMBthread);
                        obstacleAvoid.start();
                        startHoming = new Thread(oldThreads.startCombiner);
                        break;
                }

            }  else if (selectedModule == 3){
                //Selected module is optical flow, see StartScreen and start_scree.xml - RM
                //Switch to see which radio button was selected. 2 modes: Detect, and Avoid
                //Former will detect obstacle, and range and stop at a given threshold
                //Latter will detect obstacle and try and navigate around it, retaining it's path
                switch(opticalFlowModule){
                    case OF_DETECT:
                        opticalFlowThread = new Thread(threads.testThread);
                        opticalFlowThread.start();
                        break;
                    case OF_AVOID:
                        opticalFlowThread = new Thread(oldThreads.opticalFlowAvoidance);
                        opticalFlowThread.start();
                        break;
                    default: //Default to working avoidance routine
                        opticalFlowThread = new Thread(oldThreads.opticalFlowAvoidance);
                        opticalFlowThread.start();
                        break;
                    }
            } else if (selectedModule == 4){
                switch(visualNavigationModule){
                    case VN_BASE:
                        Log.i("VN: ", "Thread started");
                        real = false;
                        learning_trip = new Thread(threads.navLearning);
                        learning_trip.start();
                        memory_trip = new Thread(threads.navMemory);
                        break;
                    case VN_REAL:
                        Log.i("VN: ", "Thread started: real");
                        real = true;
                        learning_trip = new Thread(threads.navLearning);
                        learning_trip.start();
                        memory_trip = new Thread(threads.navMemory);
                    default:
                        learning_trip = new Thread(threads.navLearning);
                        learning_trip.start();
                        memory_trip = new Thread(threads.navMemory);
                }
            }


        } // if ( !opticCheck && counting > 7 )

        sec_check = true;

        // Frame count management
        counting++;
        if( counting == 20 ) counting = 0;

        //Here we put the processed image back into a Mat with the proportions of the output image
        current_image.copyTo(
                    displayedImage.rowRange(0, current_image.rows())
                                  .colRange(0, displayedImage.cols())
        );

        debugFlowImage.copyTo(
                    displayedImage.rowRange(displayedImage.rows()-debugFlowImage.rows(), displayedImage.rows())
                                  .colRange(0, displayedImage.cols())
        );

        //Resizing of the Image to fit the size of the JavaImageView
        Size size = new Size(rgba.width(), rgba.height());
        Imgproc.resize(displayedImage, displayedImage, size);

        return displayedImage;
    }


    //
    // MOVE THESE METHODS. SOMEHOW, GET THEM OUT OF THIS CLASS!!!!
    //

    private void computeSparseOpticFlow() {
        // first time around we initialize all
        // Looking at repurposing this method for optical flow obstacle avoidance - RM
        if (previous_image == null) {

            previous_image = Mat.zeros(10, 90, CvType.CV_8UC1);
            initialImage = new MatOfPoint();
            currentPointsToTrack = new MatOfPoint2f();
            prevPointsToTrack = new MatOfPoint2f();
            Status = new MatOfByte();
            Err = new MatOfFloat();

            current_image.copyTo(previous_image);
            goodFeaturesToTrack(current_image, initialImage, 100, 0.5, 3);
            initialImage.convertTo(prevPointsToTrack, CvType.CV_32FC2);
        }

        //What? Only the left flow is computed??? - RM
        //Seemingly only using the central image so regular flow is computed -- RM
        // now we can compute the flow
        // left flow

        //Seems to be an embellished if ( !(prevPointsToTrack == null) ) {}
        if (prevPointsToTrack.cols() > 0 && prevPointsToTrack.rows() > 0) {
            calcOpticalFlowPyrLK(
                    previous_image,
                    current_image,
                    prevPointsToTrack,
                    currentPointsToTrack, // variable holding the new point positions
                    Status,
                    Err,
                    new Size(9, 9),
                    2
            );
            tracked = true;
        } else {
            tracked = false;
        }
    }

    private void computeDenseOpticFlow() {
        // first time around we initialize all
        if (previous_image == null) {

            previous_image = Mat.zeros(10, 90, CvType.CV_8UC1);
            currentPointsToTrack = new MatOfPoint2f();
            Status = new MatOfByte();
            Err = new MatOfFloat();

            current_image.copyTo(previous_image);
        }
        // now we can compute the flow
        // left flow

        //Again, the flow is not computed as described in his dissertation. - RM
        // Only computed on the central frame? Not the left and right - RM
        calcOpticalFlowFarneback(
                previous_image,
                current_image,
                currentPointsToTrack,
                0.5,
                1,
                5,
                3,
                3,
                1.1,
                0
        );
        tracked = true;
    }


    private void getSpeedsFromSparseFlow() {

        debugFlowImage = Mat.zeros(rightCXFlowImage.rows()*3, rightCXFlowImage.cols(), rightCXFlowImage.type());
        int right_image_row = debugFlowImage.rows()-leftCXFlowImage.rows(); // displacement to draw the image in the right place
        leftCXFlowImage.copyTo(
                debugFlowImage.rowRange(0, leftCXFlowImage.rows())
                        .colRange(0, leftCXFlowImage.cols())
        );

        rightCXFlowImage.copyTo(
                debugFlowImage.rowRange(right_image_row, debugFlowImage.rows())
                        .colRange(0, debugFlowImage.cols())
        );

        if (tracked) {
            // compute preferred flow vectors for filtering
            Mat left_preferred_flow = CX_Holonomic.get_preferred_flow(90, Math.toRadians(currentDegree), true);
            Mat right_preferred_flow = CX_Holonomic.get_preferred_flow(90, Math.toRadians(currentDegree), false);
            int totalpoints = currentPointsToTrack.rows() * currentPointsToTrack.cols();

            float leftFlowSum = 0;
            float rightFlowSum = 0;

            Mat left_pref_vector;
            Mat right_pref_vector;
            Mat flowVector = new Mat(1, 3, CvType.CV_32FC1);

            double[] current_left_flow_vector;
            double[] previous_left_flow_vector;

            //For every point in the points of interest matrix - RM
            for (int i = 0; i < currentPointsToTrack.rows(); i++) {
                for (int j = 0; j < currentPointsToTrack.cols(); j++) {

                    // ------------------- compute left flow --------------------
                    //-----------------------------------------------------------
                    // If the current point has moved far enough between the two frames we can
                    // compute a flow vector (+12 is for left side offset)
                    if (Math.abs(
                            Util.mod((int) currentPointsToTrack.get(i, j)[0] + 12, 90)
                            - Util.mod((int) prevPointsToTrack.get(i, j)[0] + 12, 90))
                            < 70){

                        //Compute previous and current flow vectors
                        current_left_flow_vector = new double[]{Util.mod((int) currentPointsToTrack.get(i, j)[0] + 12, 90), (int) currentPointsToTrack.get(i, j)[1]};
                        previous_left_flow_vector = new double[]{Util.mod((int) prevPointsToTrack.get(i, j)[0] + 12, 90), (int) prevPointsToTrack.get(i, j)[1]};

                        //   print arrows on debugFlowImage image - DEBUG
                        org.opencv.core.Point pt1_left = new org.opencv.core.Point(
                                previous_left_flow_vector[0], previous_left_flow_vector[1]);
                        org.opencv.core.Point pt2_left = new org.opencv.core.Point(
                                current_left_flow_vector[0], current_left_flow_vector[1]);
                        arrowedLine(debugFlowImage, pt1_left, pt2_left, new Scalar(1));
                        //-----------------------------------------------------------

                        // preferred flow vector as 1x3 Mat
                        left_pref_vector = left_preferred_flow.row((int) previous_left_flow_vector[0]);

                        // ------get flow as Mat, then filter and sum up-----------
                        flowVector.put(0, 0, current_left_flow_vector[0] - previous_left_flow_vector[0]);
                        flowVector.put(0, 1, current_left_flow_vector[1] - previous_left_flow_vector[1]);
                        flowVector.put(0, 2, 0);

                        // filter vector by preferred flow and sum

                        leftFlowSum -= left_pref_vector.dot(left_pref_vector);
                        leftFlowSum += left_pref_vector.dot(flowVector);
                    }

                    // ------------------- compute right flow --------------------
                    //------------------------------------------------------------
                    // Again, compute the difference between the x coordinates to see if we have
                    // enough info to form an informative flow vector
                    if (Math.abs(
                            Util.mod((int) currentPointsToTrack.get(i, j)[0] - 12, 90)
                            - Util.mod((int) prevPointsToTrack.get(i, j)[0] - 12, 90))
                            < 70){

                        //Compute current and previous point vectors
                        double[] current_right_flow_vector = {Util.mod((int) currentPointsToTrack.get(i, j)[0] - 12, 90), (int) currentPointsToTrack.get(i, j)[1]};
                        double[] previous_right_flow_vector = {Util.mod((int) prevPointsToTrack.get(i, j)[0] - 12, 90), (int) prevPointsToTrack.get(i, j)[1]};

                        //   print arrows on debugFlowImage image - DEBUG
                        org.opencv.core.Point pt1_right = new org.opencv.core.Point(
                                previous_right_flow_vector[0],
                                previous_right_flow_vector[1] + right_image_row);
                        org.opencv.core.Point pt2_right = new org.opencv.core.Point(
                                current_right_flow_vector[0],
                                current_right_flow_vector[1] + right_image_row);
                        arrowedLine(debugFlowImage, pt1_right, pt2_right, new Scalar(1));
                        //-----------------------------------------------------------

                        // preferred flow vector as 1x3 Mat
                        right_pref_vector = right_preferred_flow.row((int) previous_right_flow_vector[0]);

                        // ------get flow as Mat then filter and sum up-----------
                        flowVector.put(0, 0, current_right_flow_vector[0] - previous_right_flow_vector[0]);
                        flowVector.put(0, 1, current_right_flow_vector[1] - previous_right_flow_vector[1]);
                        flowVector.put(0, 2, 0);

                        // filter vector by preferred flow and sum
                        rightFlowSum -= right_pref_vector.dot(right_pref_vector);
                        rightFlowSum += right_pref_vector.dot(flowVector);
                    }
                }
            }

            //(14) from Lucas' dissertation
            leftCXFlow = -1000*leftFlowSum/(totalpoints*(currentFrameTime-prevFrameTime));
            rightCXFlow = -1000*rightFlowSum/(totalpoints*(currentFrameTime-prevFrameTime));

            current_image.copyTo(previous_image); //Set next previous image
            goodFeaturesToTrack(current_image, initialImage, 100, 0.5, 3); //Get corners from current image
            initialImage.convertTo(prevPointsToTrack, CvType.CV_32FC2); // Set as previous points to track

        }
    }

    private void getSpeedsFromDenseFlow() {

        debugFlowImage = Mat.zeros(rightCXFlowImage.rows()*3, rightCXFlowImage.cols(), rightCXFlowImage.type());
        int right_image_row = debugFlowImage.rows()-leftCXFlowImage.rows(); // displacement to draw the image in the right place
        leftCXFlowImage.copyTo(
                debugFlowImage.rowRange(0, leftCXFlowImage.rows())
                        .colRange(0, leftCXFlowImage.cols())
        );

        rightCXFlowImage.copyTo(
                debugFlowImage.rowRange(right_image_row, debugFlowImage.rows())
                        .colRange(0, debugFlowImage.cols())
        );

        // compute preferred flow vectors for filtering
        Mat left_preferred_flow = CX_Holonomic.get_preferred_flow(90, Math.toRadians(0), true);
        Mat right_preferred_flow = CX_Holonomic.get_preferred_flow(90, Math.toRadians(0), false);
        int totalpoints = currentPointsToTrack.rows() * currentPointsToTrack.cols();

        float leftFlowSum = 0;
        float rightFlowSum = 0;
        Mat left_pref_vector;
        Mat right_pref_vector;
        Mat flowVector = new Mat(1, 3, CvType.CV_32FC1);
        double[] current_left_flow_vector;
        double[] previous_left_flow_vector;
        double[] current_right_flow_vector;
        double[] previous_right_flow_vector;

        for (int y = 0; y < currentPointsToTrack.rows(); y++) {
            for (int x = 0; x < currentPointsToTrack.cols(); x++) {

                // ------------------- compute left flow --------------------
                //-----------------------------------------------------------

                current_left_flow_vector = new double[]{Util.mod((int) currentPointsToTrack.get(y, x)[0] + x + 12, 90),
                                                        Util.mod((int) currentPointsToTrack.get(y, x)[1] + y, 10)};
                previous_left_flow_vector = new double[]{Util.mod(x + 12, 90), y};

                if (Math.abs(previous_left_flow_vector[0] - current_left_flow_vector[0]) < 70) {
                    //   print arrows on debugFlowImage image - DEBUG
                    /*
                    if (x + 12 == 20 || x + 12 == 70) {
                        org.opencv.core.Point pt1_left = new org.opencv.core.Point(
                                previous_left_flow_vector[0], previous_left_flow_vector[1]);
                        org.opencv.core.Point pt2_left = new org.opencv.core.Point(
                                current_left_flow_vector[0], current_left_flow_vector[1]);
                        arrowedLine(debugFlowImage, pt1_left, pt2_left, new Scalar(0));
                    }*/
                    //-----------------------------------------------------------

                    // preferred flow vector as 1x3 Mat
                    left_pref_vector = left_preferred_flow.row((int) previous_left_flow_vector[0]);

                    // ------get flow as Mat, then filter and sum up-----------
                    flowVector.put(0, 0, currentPointsToTrack.get(y, x)[0]);
                    flowVector.put(0, 1, currentPointsToTrack.get(y, x)[1]);
                    flowVector.put(0, 2, 0);

                    // filter vector by preferred flow and sum
                    leftFlowSum += left_pref_vector.dot(flowVector);
                }


                // ------------------- compute right flow --------------------
                //-----------------------------------------------------------

                current_right_flow_vector = new double[] {Util.mod((int) currentPointsToTrack.get(y, x)[0] + x - 12, 90),
                                                          Util.mod((int) currentPointsToTrack.get(y, x)[1] + y, 10)};
                previous_right_flow_vector = new double[] {Util.mod(x - 12, 90), y};

                if (previous_right_flow_vector[0] - current_right_flow_vector[0] < 70) {
                    //   print arrows on debugFlowImage image - DEBUG
                    /*
                    if (x - 12 == 20 || x - 12 == 70) {
                        org.opencv.core.Point pt1_right = new org.opencv.core.Point(
                                previous_right_flow_vector[0],
                                previous_right_flow_vector[1] + right_image_row);
                        org.opencv.core.Point pt2_right = new org.opencv.core.Point(
                                current_right_flow_vector[0],
                                current_right_flow_vector[1] + right_image_row);
                        arrowedLine(debugFlowImage, pt1_right, pt2_right, new Scalar(0));
                    }*/
                    //-----------------------------------------------------------

                    // preferred flow vector as 1x3 Mat
                    right_pref_vector = right_preferred_flow.row((int) previous_right_flow_vector[0]);

                    // ------get flow as Mat then filter and sum up-----------
                    flowVector.put(0, 0, currentPointsToTrack.get(y, x)[0]);
                    flowVector.put(0, 1, currentPointsToTrack.get(y, x)[1]);
                    flowVector.put(0, 2, 0);

                    // filter vector by preferred flow and sum
                    rightFlowSum += right_pref_vector.dot(flowVector);
                }
            }
        }
        //swap to account for image shift (i.e. image shifted right gives left flow and vice-versa)
        leftCXFlow = 1000*rightFlowSum/(totalpoints*(currentFrameTime-prevFrameTime));
        rightCXFlow = 1000*leftFlowSum/(totalpoints*(currentFrameTime-prevFrameTime));

        current_image.copyTo(previous_image);
    }

    public void filterCollisionAvoidance() {
        global_current_time = (int) SystemClock.elapsedRealtime() - global_start_time; //Time since start

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

        for ( int y = 0; y < currentPointsToTrack.rows(); y++ ){
            for ( int x = 0; x < currentPointsToTrack.cols(); x++ ){//These filters are functionally identical, see Luca's dissertation for the method used to compute the filter
                //Compute left flow vector
                //+-4 is the left/right offset for the flow frame; e.g. -4 centres the frame at -16deg.
                //The narrower this is, the closer we are to considering the original flow filter.
                //So this may effectively act as our rather than the raw pixel values.
                previous_left_flow_vector = new double[]{Util.mod(x + offset, 90), y}; //((x+12), y)
                current_left_flow_vector = new double[]{Util.mod((int) currentPointsToTrack.get(y, x)[0] + x + offset, 90),
                        Util.mod((int) currentPointsToTrack.get(y, x)[1] + y, 10)}; //Flow info returned by farneback

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
                current_right_flow_vector = new double[] { Util.mod((int) currentPointsToTrack.get(y,x)[0] + x - offset, 90),
                        Util.mod((int) currentPointsToTrack.get(y,x)[1] + y, 10) };

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
        rightCAFlow =  1000 * left_flow_sum;// / (delta /*900*/);
        leftCAFlow = 1000 * right_flow_sum; // / (delta /*900*/);

    }

    //
    // END OF ERRONEOUS METHODS
    //

    public class SaveImages extends AsyncTask<Object, Integer, Boolean> {
        @Override
        protected Boolean doInBackground(Object... transmission) {
            Mat matImage = (Mat) transmission[0];
            //Since there is no possibility to send a 2D array or Mat file the image is transformed to a 1D int array
            byte[] imageArray_tmp = new byte[matImage.height() * matImage.width()];
            byte[] revArray_tmp = new byte[matImage.height() * matImage.width()];
            matImage.get(0, 0, imageArray_tmp);

            //imageArray_tmp = rotateMatInAzimuth(0, matImage); //For curiosity
            revArray_tmp = Util.rotateMatInAzimuth(45, matImage); //Array of reverse image (for homeward route)

            int[] imageArray = new int[imageArray_tmp.length];
            int[] revImageArray = new int[imageArray_tmp.length];
            String image_string="";
            for (int n = 0; n < imageArray_tmp.length; n++) {
                imageArray[n] = (int) imageArray_tmp[n] & 0xFF;
                revImageArray[n] = (int) revArray_tmp[n] & 0xFF;
                image_string += imageArray[n]+ " ";
            }
            //model.onNewImage... for Zhaoyu's code
            mushroom_body.onNewImage(imageArray, (int) transmission[1]); //Learn image,
            mushroom_body.onNewImage(revImageArray, (int) transmission[1]); //Learn reversed image
            //StatFileUtils.write("SI", "MB", "Image: " + image_string);
            // LogToFileUtils.write("IMAGE: " + image_string);
            // LogToFileUtils.write("REQUEST CODE: " + (int) transmission[1]);
            return false;
        }
    }

    public class LearnDirectionImages extends AsyncTask<Object, Integer, Boolean> {

        @Override
        protected Boolean doInBackground(Object... transmission) {
            Mat matImage = (Mat) transmission[0];
            //Since there is no possibility to send a 2D array or Mat file the image is transformed to a 1D int array
            byte[] imageArray_tmp = new byte[matImage.height() * matImage.width()];
            matImage.get(0, 0, imageArray_tmp);
            int[] imageArray = new int[imageArray_tmp.length];

            for (int n = 0; n < imageArray_tmp.length; n++) {
                imageArray[n] = (int) imageArray_tmp[n] & 0xFF;
            }
            int request_code = (int) transmission[1];
            if (request_code == 7){
                cxmb.setupLearningAlgorithm(imageArray);
            } else {
                SimpleMatrix tb1 = (SimpleMatrix) transmission[2];
                cxmb.learnImageWithDirection(imageArray, tb1);
            }

            LogToFileUtils.write("REQUEST CODE: " + (int) transmission[1]);
            return false;
        }
    }

    private class Startback extends AsyncTask<Object, Integer, Boolean> {
        @Override
        protected Boolean doInBackground(Object... transmission) {
            startHoming.start();
            return true;
        }
    }
}