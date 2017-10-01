package insectsrobotics.imagemaipulations;

import android.app.Activity;
import android.content.ComponentName;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.ServiceConnection;
import android.graphics.Color;
import android.graphics.Point;
import android.hardware.Camera;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorManager;
import android.hardware.SensorEventListener;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Environment;
import android.os.Handler;
import android.os.IBinder;
import android.os.SystemClock;
import android.util.Log;
import android.view.Display;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.ProgressBar;
import android.widget.TextView;
import android.graphics.Bitmap;

// Zhaoyu added this for put back to food alert
import android.app.AlertDialog;
import android.content.DialogInterface;


import org.ejml.simple.SimpleMatrix;
import org.opencv.android.Utils;
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
import org.opencv.core.MatOfDouble;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.io.FileOutputStream;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Currency;
import java.util.List;
import java.io.OutputStreamWriter;

import insectrobotics.broadcastlibrary.BroadcastValues;
import insectsrobotics.imagemaipulations.Receiver_and_Broadcaster.Broadcast;
import insectsrobotics.imagemaipulations.Receiver_and_Broadcaster.Receive;
import insectsrobotics.imagemaipulations.NavigationModules.WillshawModule;
import insectsrobotics.imagemaipulations.NavigationModules.PerfectMemoryModule;

import static java.lang.Thread.sleep;
import static org.opencv.imgproc.Imgproc.COLOR_BayerRG2RGB_EA;
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
    private static final String TAG = "OCVSample::Activity";
    boolean opticCheck = false;
    public Mat processedSourceImage;
    public Mat processedDestImage;
    public Mat displayedImage;
    Button learnImageBtn;
    Button calcErrorBtn;
    int counting =0;
    Mat imageToDisplay;
    Mat imageToCompare;
    Mat denseFlowPoints;
    Mat pastFlow;
    //Receiver and Broadcaster
    Broadcast broadcast;
    Receive receive;
    //Image processing variables
    Mat BlueChannel;
    MatOfInt from_to;
    Mat rgba;
    List<Mat> rgbaList;
    List<Mat> BlueChannelList;
    double[] pixel;
    String myTag ="Canny";
    boolean check = false;
    String flowTag="flow";
    //AdvancedSettings Data
    Bundle advancedSettingsBundle;
    Mat[] storedPics = new Mat[4];

    // ----- CX variables ------
    int startTime;
    float frame_rate_cx = 0;
    private double currentDegree = 0.;
    private double CXnewHeading;
    private double CXtheta;
    private double CXmotor = 0;
    private double CXmotorChange = 0.5;
    double ANT_SPEED = 1.;
    SimpleMatrix holonominc_speed = new SimpleMatrix(new double[][] {{1.},{1.}});
    private SensorManager mSensorManager;
    private Sensor mSensor;
    private int CURRiteration;

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
    int flowTime = 0;

    int cfn = 0;

    //------

//    TextView tvHeading;

    //Camera Parameters
    double imageCorrection;
    int cameraViewStartedWidth = 0;
    int cameraViewStartedHeight = 0;


    //Calibration resulting variables
    Bundle mBundle;
    double[] theta_new = new double[40];
    int[][][] directoryArray = new int[(theta_new.length + 1)][360][2];


    //Unwrapping variables
    int sourceResolution = 1;
    int resolution = 4;

    //Optical flow variables
    Mat first_frame=null;
    Mat second_frame;
    double opticalSumLimit =50;


    //Background processor
    AsyncTask<Object, Integer, Boolean> broadcastImage = null;


    //Module Selection from StartScreen
    String visualModule;
    String pathIntegratorModule;
    String combinerModule;
    boolean notification = true;

    //Layout Views
    TextView serialConnectionTextView;
    //TextView serverConnectionTextView;
    TextView debugTextView;
    ProgressBar serialProgressBar;
    ProgressBar serverProgressBar;
    ImageView serialCheckImageView;
    ImageView serverCheckImageView;
    AlertDialog.Builder builder;

    //DEBUG variables to save and load images from txt files
    boolean saveImages = false;
    boolean loadImages = false;
    int numberOfImages = 0;
    int errorCounter = 0;
    int numberOfErrorCalculations = 50;
    int errorCounter100 = 0;
    Handler learnHandler = new Handler();
    File sdCard = Environment.getExternalStorageDirectory();
    File dir = new File(sdCard.getAbsolutePath() + "/");
    File file = new File(dir, "text.txt");

    //*** Camera calibration data
    double R1 = 104.7; //radius of the inner circle
    double R2 = 217.2; //radius of the outer circle
    double centreX = 527; // X coordinate of the image centre
    double centreY = 406; // Y coordiate of the image centre
    double[] affine = new double[]{1.000329,0.000171,-0.000339,1.0}; // Affine parameters c, d, e and 1
    double[] polynom = new double[]{-141.742948747035,0,-0.000022888213756,0.000010694034632,0.000000001664105}; // polynomial coefficients
    Mat affineMat;

    //** Unwrapped image parameters
    double unwrapHeight = 0;
    double unwrapWidth = 0;
    Mat imageMapX;
    Mat imageMapY;
    double scale =1;

    MatOfPoint initial;
    MatOfByte status;
    MatOfFloat err;
    MatOfPoint2f prevPts;
    MatOfPoint2f nextPts;
    Mat mGray1 = null;
    Mat mGray2;
    double leftFlow=0;
    double rightFlow=0;
    double centreFlow=0;
    double prevLeft=0;
    double prevRight=0;
    double prevCentre=0;

    double xTotal=0;
    boolean beginTurn=false;
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
    Thread runDown;
    Thread turnToMinimum;
    Thread startSearching;
    Thread search;

    //PI
    Thread obstacleAvoid;
    Thread optThread;
    Thread denseOpt;
    Thread outboundStart;
    int direction=0;
    int integratorRunTime=35000; //outbound time (milliseconds)
    boolean outboundStop =false;

    Thread testThread;
    int selectedModule=0;

    //Combiner
    int MethodChosen;
    // Choose from one of the following models for route following
    WillshawModule model = new WillshawModule();
    // PerfectMemoryModule model = new PerfectMemoryModule();

    // The combined CX and MB model
    CX_MB cxmb = new CX_MB();
    // Stored memory to retrieve the CPU memory again
    SimpleMatrix stored_memory = new SimpleMatrix(CX.n_cpu4, 1);

    AsyncTask<Object, Integer, Boolean> new_image = null;
    AsyncTask<Object, Integer, Boolean> start_home = null;
    double familarity;
    Boolean images_access = false;
    double[] familarity_array = new double[11];
    double distance_travelled;
    Thread chosen_thread;


    //Optical flow obstacle detection - RM
    //Will reuse some visual variables

    //Single thread will be used for both runnables: Detection and avoidance
    Thread opticalFlowThread; //Bot will detect obstacle and try to navigate it


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

    StringBuilder errorString = new StringBuilder();
    StringBuilder perfectMemoryErrorString = new StringBuilder();

    //See learnButton FListener
    Runnable learnRunnable = new Runnable() {
        @Override
        public void run() {
            if (!saveImages && !loadImages) {
                errorCounter = 0;
            } else {
                errorCounter = numberOfErrorCalculations;
            }
            //learnImage = true;
            //transmissionRunning = false;
            notification = true;
            errorString = new StringBuilder();
            perfectMemoryErrorString = new StringBuilder();
            calcErrorBtn.setVisibility(View.VISIBLE);
        }
    };


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
                    //Log.i(myTag, "size: "+ imageMap[0].length +" X "+ imageMap[1].length);
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
            /*else if (action.equals(STATUS_UPDATE)){
                visualNavigationRunning = intent.getBooleanExtra("MainData", false);
            }*/
        }

        @Override
        public void onNewMessageFromCombinerApp(Intent intent, String action) {
            switch (action){
                case TURN_AE_DATA:
                    try {
                        turnAround(intent.getDoubleExtra("MainData",0));
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    break;
            }
        }
        @Override
        public void onNewMessageFromIntegratorApp(Intent intent, String action) {

        }
        @Override
        public void onNewMessageFromAntEyeApp(Intent intent, String action) {
        }
    };

    // Zhaoyu added this to
    // Show the alert dialog after first run.
    private void showDialog() {

        builder=new AlertDialog.Builder(this);

        builder.setTitle(R.string.put_back_title);
        builder.setMessage(R.string.place_back_food);

        builder.setPositiveButton(R.string.yes, new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialogInterface, int id) {
                start_home = new Startback();
                start_home.execute();
            }
        });
        AlertDialog dialog=builder.create();
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
        IntentFilter intentFilter = receive.getIntentFilter();
        registerReceiver(receive, intentFilter);

        //Inflate Layout and initiate the Views
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        setContentView(R.layout.activity_image_manipulations);
        mOpenCvCameraView = (CameraBridgeViewBase) findViewById
                (insectsrobotics.imagemaipulations.Build.R.id.main_activity_surface_view);
        mOpenCvCameraView.setCvCameraViewListener(this);

        mSensorManager = (SensorManager) getSystemService(this.SENSOR_SERVICE);
        mSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR);


        serialConnectionTextView = (TextView) findViewById(R.id.serialConnectionTextView);
        //serverConnectionTextView = (TextView) findViewById(R.id.serverConnectionTextView);
        debugTextView = (TextView) findViewById(R.id.debugTextView);
        serialProgressBar = (ProgressBar) findViewById(R.id.serialProgressBar);
        //serverProgressBar = (ProgressBar) findViewById(R.id.serverProgressBar);
        serialCheckImageView = (ImageView) findViewById(R.id.serialCheckImageView);
        //serverCheckImageView = (ImageView) findViewById(R.id.serverCheckImageView);
        serialCheckImageView.setImageResource(R.drawable.checksymbol);
        //serverCheckImageView.setImageResource(R.drawable.checksymbol);


        //Loads calibration_layout data from passed Intent (Coming from StartScreen) and Advanced Settings
        Intent intent = getIntent();
        //Get Advanced Settings selection:
        //advancedSettingsBundle = intent.getBundleExtra("ServerConnection");
        //Log.e(TAG,"ServerAddress: " + advancedSettingsBundle.get("ServerAddress"));
        //Get Data from Intent
        mBundle = intent.getBundleExtra("Data");
        //Get Module Selection from Bundle
        visualModule = mBundle.getString(VN_MODULE, NO_MODULE);
        pathIntegratorModule = mBundle.getString(PI_MODULE, NO_MODULE);
        combinerModule = mBundle.getString(C_MODULE, NO_MODULE);
        selectedModule=mBundle.getInt("selectModule",0);

        // DEBUG LOG to file
        LogToFileUtils.init(this.getApplicationContext());

        //DEBUG: Create new File for error output
        try {
            FileOutputStream f = new FileOutputStream(file);
            f.write(("Measurements: " + "\n").getBytes());
        } catch (Exception e) {
            e.printStackTrace();
        }

    }

    //This has to be done since openCV crops images before the go in the "onNewFrame"-Method.
    //Therefore we have to adjust the View to the Camera parameters of the phone
    private void setCameraViewSize() {
        //Get Camera parameters (0 = BackCamera, 1= FrontCamera)
        Camera mCamera = Camera.open(1);
        Camera.Parameters params = mCamera.getParameters();
        Camera.Size pictureSize = params.getPictureSize();
        int pictureHeight = pictureSize.height;
        int pictureWidth = pictureSize.width;
        mCamera.release();

        Log.d(TAG, "Picture Height = " + pictureHeight + " Width = " + pictureWidth);

        //Used to set the Camera View accordingly
        Display display = getWindowManager().getDefaultDisplay();
        Point size = new Point();
        display.getSize(size);
        double width = size.x;

        //openCv also turns the image by 90°, so here the View is adjusted to the new parameters
        mOpenCvCameraView.getLayoutParams().height = (int) (width / (pictureWidth / pictureHeight));
        imageCorrection = pictureWidth;
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
//        double c = -2.0f * (x * z + w * y);
//        double d = +2.0f * (y * z - w * x);
//        double e = -2.0f * (x * x + ysqr) + 1.0f;

//        c = c > 1.0f ? 1.0f : c;
//        c = c < -1.0f ? -1.0f : c;

//        double pitch = Math.asin(c);
//        double roll = Math.atan2(d, e);
        double yaw = Math.atan2(b, a);
        double angle = Math.round(Math.toDegrees(yaw));
        if (angle<0){
            angle +=360;
        }
        currentDegree = angle;
//        tvHeading.setText("Heading: " + Float.toString(degree) + " degrees");
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // not in use
    }



    /**
     * Enables view again on resume of the activity
     */
    @Override
    public void onResume() {

        super.onResume();
        Log.d("onResume", "onResume called");
        imageCorrection = 0;
        //transmissionRunning = false;
        //visualNavigationRunning = false;

        //setCameraViewSize();

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
        stopThread=true;
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    /**
     * Called after successful initiation of the camera connection, initiates Variables, sets up
     * a connection array between the "Donut-View" and the unwrapped image.
     */

    public void onCameraViewStarted(int width, int height) {
        Log.e(TAG, "onCameraViewStarted called");

        //Send chosen modules
        //broadcast.broadcastModules(WILLSHAW);
        //broadcast.broadcastServerConnection(advancedSettingsBundle);

        cameraViewStartedWidth = width;
        cameraViewStartedHeight = height;

        //broadcastImage = new BroadcastImage();
    }

    public void onCameraViewStopped() {
        //optThread.interrupt();

    }

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


        for(int x=0;x<affineMat.rows();x++){
            for(int y=0;y<affineMat.cols();y++){

                affineMat.put(x,y,affine[l]);
                l++;
            }
        }

        for(int y=0; y<(int)unwrapHeight; y++){
            for(int x=0; x<(int)unwrapWidth; x++){
                r = ((double)y/unwrapHeight)*(R2-R1)+R1;
                theta = ((double)x/ unwrapWidth) * 2 * Math.PI;
                xS = (int) (centreX + r*Math.sin(theta));
                yS = (int) (centreY + r*Math.cos(theta));

                temp.put(0, 0, xS);
                temp.put(1, 0, yS);
                Core.gemm(affineMat.inv(),temp,1,new Mat(),0,temp,0);
                xS=(int)(temp.get(0,0)[0]);
                yS=(int)temp.get(1,0)[0];


                imageMapX.put(y, x, xS);
                imageMapY.put(y,x,yS);

            }
        }
    }

    // method to save the frames on the DCIM folder in the android phone.
    public void saveImageToFolder(Mat image, String filename) {

        Bitmap bmp = null;
        try {
            bmp = Bitmap.createBitmap(image.cols(), image.rows(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(image, bmp);
        } catch (Exception e) {
            Log.d(TAG, e.getMessage());
        }

        FileOutputStream out = null;
        filename += ".png";

        File sd = new File(Environment.getExternalStorageDirectory(),"/DCIM/");
        boolean success = true;
        if (!sd.exists()) {
            success = sd.mkdir();
        }
        if (success) {
            File dest = new File(sd, filename);

            try {
                out = new FileOutputStream(dest);
                bmp.compress(Bitmap.CompressFormat.PNG, 100, out); // bmp is your Bitmap instance
                // PNG is a lossless format, the compression factor (100) is ignored

            } catch (Exception e) {
                e.printStackTrace();
                Log.d(TAG, e.getMessage());
            } finally {
                try {
                    if (out != null) {
                        out.close();
                        Log.d(TAG, "OK!!");
                    }
                } catch (Exception e) {
                    Log.d(TAG, e.getMessage() + "Error");
                    e.printStackTrace();
                }
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
    public Mat onCameraFrame(CvCameraViewFrame inputFrame) {

        //Initiation of the needed Variables
        rgbaList = new ArrayList<>();
        BlueChannelList = new ArrayList<>();
        from_to = new MatOfInt(2, 0);
        rgba = new Mat();

        // initialize some CX variables
        currentRightCXImage = Mat.zeros(10, 90, CvType.CV_8UC1);
        currentLeftCXImage = Mat.zeros(10, 90, CvType.CV_8UC1);
        leftCXFlowImage = Mat.zeros(10, 90, CvType.CV_8UC1);
        rightCXFlowImage = Mat.zeros(10, 90, CvType.CV_8UC1);


        current_image = Mat.zeros(10, 90, CvType.CV_8UC1);
        debugFlowImage = Mat.zeros(10, 90, CvType.CV_8UC1);

        processedSourceImage = Mat.zeros(theta_new.length / sourceResolution, 360 / sourceResolution, CvType.CV_8UC1);

        processedDestImage = Mat.zeros(theta_new.length / resolution, 360 / resolution, processedSourceImage.type());
        images_access = false;
        Mat tempRotate = Mat.zeros(processedSourceImage.size(), processedSourceImage.type());
        fullImageToDisplay=Mat.zeros(processedSourceImage.size(),processedSourceImage.type());


        prevFrameTime = currentFrameTime;
        currentFrameTime = (float) SystemClock.elapsedRealtime();
        if ((currentFrameTime-prevFrameTime)/1000 > 0){
            frame_rate_cx = 1/((currentFrameTime-prevFrameTime)/1000);
        } else {
            frame_rate_cx = 0;
        }


        rgba = inputFrame.rgba();                                           //Input Frame in rgba format
        BlueChannel = new Mat(rgba.rows(), rgba.cols(), CvType.CV_8UC1);    //Mat for later image processing

        Mat unwrappedImg =  Mat.zeros((int) unwrapHeight, (int) unwrapWidth, CvType.CV_8UC1);

        double scale = rgba.width() / (360 / resolution);
        double rgbaHeight = rgba.height();
        displayedImage = Mat.zeros((int) (rgbaHeight / scale), 360 / resolution, CvType.CV_8UC1);

        rgbaList.add(rgba);                                                 //Needed for channel extraction from rgba image
        BlueChannelList.add(BlueChannel);                                   //Needed for channel extraction from rgba image
        Core.mixChannels(rgbaList, BlueChannelList, from_to);               //Extract only the blue channel from rgba

        remap(BlueChannel, unwrappedImg, imageMapX, imageMapY, INTER_LINEAR);
        resize(unwrappedImg, tempRotate, processedSourceImage.size(), 2.81, 2.81, INTER_LANCZOS4);

        int counter1 = 0;
        int counter3;
        int colPosition;

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


        //Mat serverImage = new Mat(processedSourceImage.size(),processedSourceImage.type());
        //Core.flip(processedSourceImage,serverImage, -1);
        //broadcast.broadcastImageForServer(serverImage);

        /**
         * If the output resolution is different from 1x1° we have to down-sample the image.
         * openCVs RegionOfInterest method is perfect for this cause.
         *
         */

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

        }
        /**
         * If the output resolution is the same as the source resolution, we can use the same Image
         */
        else {
            processedDestImage = processedSourceImage;
        }

        Imgproc.equalizeHist(processedDestImage, processedDestImage);
        GaussianBlur(processedDestImage, processedDestImage, new Size(3, 3), 0, 0);

        images_access = true;
        //broadcastImage = new BroadcastImage();
        //processedDestImage.copyTo(visual_image);
        //broadcastImage.execute(processedDestImage);


        //shift image so that left side is left and right side is right
        processedDestImage.colRange(0, 68).copyTo(current_image.colRange(22, 90));
        processedDestImage.colRange(68, 90).copyTo(current_image.colRange(0, 22));

        // takes 360 image with center at 45 deg for left flow
        current_image.colRange(12, 90).copyTo(rightCXFlowImage.colRange(0, 78));
        current_image.colRange(0, 12).copyTo(rightCXFlowImage.colRange(78, 90));

        // takes 360 image with center at -45 deg for right flow
        current_image.colRange(0, 78).copyTo(leftCXFlowImage.colRange(12, 90));
        current_image.colRange(78, 90).copyTo(leftCXFlowImage.colRange(0, 12));

//      compute optic flow and charge the global variables with the speed values

//        computeSparseOpticFlow();
//        getSpeedsFromSparseFlow();

        computeDenseOpticFlow();
        getSpeedsFromDenseFlow();

        // --commented for speed--
//        processedDestImage.copyTo(second_frame);

        // Uncommented by Zhaoyu
        //processedSourceImage.copyTo(fullImageToDisplay);
        //processedDestImage.copyTo(imageToDisplay);

        if(!opticCheck && counting>7){
            temporaryImageToCompare=Mat.zeros(processedDestImage.size(),processedDestImage.type());
            opticCheck=true;
            fullSnapShot=Mat.zeros(processedSourceImage.size(),processedSourceImage.type());
            snapShot=Mat.zeros(processedDestImage.size(),processedDestImage.type());

            processedDestImage.copyTo(snapShot);
            processedSourceImage.copyTo(fullSnapShot);
            rotatedImageDB=new ArrayList<>();

            Log.i(flowTag, "ModuleSelected"+selectedModule);

            if(selectedModule==0){
                // Uncomment to start recular CX (one speed input)
                //obstacleAvoid=new Thread(CXthread);
                // Uncomment to start Holonomic CX (two speed inputs)
                obstacleAvoid=new Thread(CXHolonomicThread);
                obstacleAvoid.start();
            }else if (selectedModule == 1){ //If selected module is for visual navigation
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
            } else { //Else run the combiner module
                switch(combinerModule){
                    case BACK_WITH_MB:
                        MethodChosen = 0;
                        obstacleAvoid=new Thread(CXthread);
                        obstacleAvoid.start();
                        startHoming = new Thread(startScanning);
                        break;
                    case KLINOKINESIS:
                        MethodChosen = 1;
                        obstacleAvoid=new Thread(CXthread);
                        obstacleAvoid.start();
                        startHoming = new Thread(startKlinokinesis);
                        break;
                    case EIGHT_ENS:
                        MethodChosen = 2;
                        obstacleAvoid = new Thread(CXMBthread);
                        obstacleAvoid.start();
                        startHoming = new Thread(startCombiner);
                        break;
                }

            }


        }
        sec_check = true;
        counting++;
        if(counting==20) counting=0;

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



    //------------------

    //Rotating the 360X40 image, then down-sample to 90X10
    private Mat rotateFullImage(Mat pic, int angle){
        Mat processedMat;
        processedMat=Mat.zeros(processedDestImage.size(),processedDestImage.type());
        int counter1;
        int counter3=0;
        int colPosition;
        Mat tempRotate;
        double[] pixelNew;

        tempRotate=new Mat(fullSnapShot.size(),fullSnapShot.type());
        for (int theta_tmp = 0; theta_tmp < 40; theta_tmp = theta_tmp + 1) {
            counter1 = 0;
            for (int phi = 0; phi < 360; phi = phi + 1) {
                colPosition=counter1+angle;
                if(colPosition>=360){
                    colPosition=colPosition - 360;
                }
                pixelNew = pic.get(counter3,colPosition);
                tempRotate.put(counter3, counter1, pixelNew);

                counter1++;
            }
            counter3++;
        }

        //downsampling
        int destAzimuthCounter = 0;
        for (int azimuth = 0; azimuth < tempRotate.cols(); azimuth = azimuth + 4) {
            int destElevationCounter = 0;
            for (int elevation = 0; elevation < tempRotate.rows(); elevation = elevation + 4) {
                Rect roi;
                //New Rectangle with the target resolution, later the ROI of the frame.
                roi = new Rect(azimuth, elevation, 4, 4);
                if (tempRotate.cols() - azimuth < 4) {
                    roi = new Rect(azimuth, elevation, tempRotate.cols() - azimuth, 4);
                }
                if (tempRotate.rows() - elevation < 4) {
                    roi = new Rect(azimuth, elevation, 4, tempRotate.rows() - elevation);
                }
                //Getting the pixels of the Region of Interest and averaging the values.
                Mat ROI = tempRotate.submat(roi);
                int ROIMean = (int) Core.mean(ROI).val[0];
                processedMat.put(destElevationCounter, destAzimuthCounter, ROIMean);
                destElevationCounter++;

            }
            destAzimuthCounter++;
        }

        Imgproc.equalizeHist(processedMat, processedMat);

        GaussianBlur(processedMat, processedMat, new Size(3, 3), 0, 0);
        broadcast.broadcastRotatedImage(processedMat);
        return processedMat;
    }

    //Rotate the 360X40 snapshot
    private void rotateSnapShot(){
        int counter1 = 0;
        int counter3;
        int colPosition;
        Mat tempRotate;
        tempRotate=Mat.zeros(fullSnapShot.size(),fullSnapShot.type());
        for (int phi = 0; phi < 360; phi = phi + 1) {
            counter3 = 0;
            for (int theta_tmp = 0; theta_tmp < 40; theta_tmp = theta_tmp + 1) {
                colPosition=counter1+180;
                if(colPosition>=360){
                    colPosition=colPosition - 360;
                }
                double[] pixelNew = fullSnapShot.get(counter3,colPosition);
                tempRotate.put(counter3, counter1, pixelNew);
                counter3++;
            }
            counter1++;
        }
        tempRotate.copyTo(fullSnapShot);

    }

    // Monitoring time to stop outbound PI and start Inbound

    Runnable startInbound = new Runnable() {
        @Override
        public void run() {
            try {
                sleep(integratorRunTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    };


    Runnable CXthread=new Runnable() {
        @Override
        public void run() {
            try {
                sleep(3000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            //Log.i(flowTag, "" + xPosOF + "," + yPosOF+","+orientation)
            CX c = new CX();
            Integer T_outbound = 40;
            Integer T_inbound = 30;//55;
            Integer T = T_inbound + T_outbound;

            SimpleMatrix tl2 = new SimpleMatrix(CX.n_tl2, 1);
            tl2.set(0);
            SimpleMatrix cl1 = new SimpleMatrix(CX.n_cl1, 1);
            cl1.set(0);
            SimpleMatrix tb1 = new SimpleMatrix(CX.n_tb1, 1);
            tb1.set(0);
            SimpleMatrix memory = new SimpleMatrix(CX.n_cpu4, 1);
            memory.set(.5);
            SimpleMatrix cpu4 = new SimpleMatrix(CX.n_cpu4, 1);
            cpu4.set(0);
            SimpleMatrix cpu1 = new SimpleMatrix(CX.n_cpu1, 1);
            cpu1.set(0);

            startTime = (int) SystemClock.elapsedRealtime();
            int t0 = (int) SystemClock.elapsedRealtime();
            int t = 0;
            CURRiteration = 0;
            int currentTime = (int)SystemClock.elapsedRealtime()-startTime;
            while (currentTime<T_outbound*1000) {
                CURRiteration++;
                try {
                    sleep(600);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                try {
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            debugTextView.setText(String.format("outbound " +
                                    "\nLeft Speed: %f"+
                                    "\nRight Speed: %f"+
                                    "\ncurrent iteration: %d"+
                                    "\ncurrent direction: %f",
                                    leftCXFlow,
                                    rightCXFlow,
                                    CURRiteration,
                                    currentDegree));
                        }
                    });
                } catch (Exception e) {
                    e.printStackTrace();
                }

                // OUTBOUND ROUTE
                t = (int) SystemClock.elapsedRealtime() - t0;

                String direction = "not sure";
                if(t>300) {
                    if (currentTime < 3500) {
                        go(new double[]{100, 100});
                        ANT_SPEED = 4.0;
                        direction = "straight";
                    } else if (currentTime < 8000) {
                        go(new double[]{10, 100});
                        ANT_SPEED = 2.0;
                        direction = "left";
                    } else if (currentTime < 11000) {
                        go(new double[]{100, 10});
                        ANT_SPEED = 2.0;
                        direction = "left";
                    } else if (currentTime < 25000) {
                        go(new double[]{100, 100});
                        ANT_SPEED = 4.;
                        direction = "straight";
                    } else {
                        go(new double[]{100, 10});
                        ANT_SPEED = 4.;
                        direction = "straight";
                    }
                    // based on odometry
                    //ANT_SPEED = ()

                    t0 = (int) SystemClock.elapsedRealtime();
                    Log.d(TAG, "facing " + currentDegree + ", going " + direction);
                    Log.d(TAG, printMemory(memory));
                }
                // updates in memory structure as we move which will allow return to nest
                //------   COMPASS UPDATE  -----
                tl2 = c.tl2Output(Math.toRadians(currentDegree));
                cl1 = c.cl1Output(tl2);
                tb1 = c.tb1Output(cl1, tb1);

                // ------ DISPLACEMENT UPDATE -----
                memory = c.cpu4Update(memory, tb1, ANT_SPEED);
                cpu4 = c.cpu4Output(memory.copy());

                // ----- TURNING GENERATION ------
                cpu1 = c.cpu1Output(tb1, cpu4);
                CXmotor = c.motorOutput(cpu1);
                writeToFile(memory, leftCXFlow, rightCXFlow, "run_1");
                currentTime = (int) SystemClock.elapsedRealtime()-startTime;
            }

            // turn to offset pull, and allow network to redirect antbot
            go(new double[]{0, 0});
            try {
                sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            try{
                turnAround(45);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            currentTime = (int)SystemClock.elapsedRealtime()-t0;

            startTime = (int) SystemClock.elapsedRealtime();
            t0 = (int)SystemClock.elapsedRealtime();
            t = 0;

            Boolean algorithm_not_setted = true;

            while (algorithm_not_setted){
                if (images_access){
                    new_image = new SaveImages();
                    new_image.execute(processedDestImage, 7);
                    algorithm_not_setted = false;
                }
            }

            // INBOUND ROUTE
            while (currentTime < T_inbound*1000){
                CURRiteration++;
                try {
                    sleep(900);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                if (images_access) {
                    new_image = new SaveImages();
                    new_image.execute(processedDestImage, 5);
                }

                //------   COMPASS UPDATE  -----
                tl2 = c.tl2Output(Math.toRadians(currentDegree));
                cl1 = c.cl1Output(tl2);
                tb1 = c.tb1Output(cl1, tb1);

                // ------ DISPLACEMENT UPDATE -----
                memory = c.cpu4Update(memory, tb1, ANT_SPEED);
                cpu4 = c.cpu4Output(memory.copy());

                // ----- TURNING GENERATION ------
                cpu1 = c.cpu1Output(tb1, cpu4);
                CXmotor = c.motorOutput(cpu1);
                writeToFile(memory, leftCXFlow, rightCXFlow, "run_1");

                // ----- ANTBOT DRIVING--------
                CXnewHeading = Math.toDegrees(Math.toRadians(currentDegree) - CXmotorChange * CXmotor);
                CXtheta = (CXnewHeading - currentDegree)%360;

                t = (int)SystemClock.elapsedRealtime() - t0;
                String direction = "not sure";
                if(t > 300){
                // speed in dm/sec
                    if (CXtheta<-1.5){
                        go(new double[]{10, 100});
                        ANT_SPEED = 0.5;
                        direction = "left";
                    } else if (CXtheta>1.5){
                        go(new double[]{100, 10});
                        ANT_SPEED = 0.5;
                        direction = "right";
                    } else {
                        go(new double[]{100, 100});
                        ANT_SPEED = 4.;
                        direction = "straight";
                    }
                    Log.d(TAG, "facing " + currentDegree + ", going " + direction + "\nwant to go to " + CXnewHeading);
                    t0 = (int) SystemClock.elapsedRealtime();
                    Log.d(TAG, printMemory(memory));
                }

                try {
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            debugTextView.setText(String.format("inbound " +
                                    "\ncurrent iteration: %d"+
//                                    "\ncurrent direction: %f"+
                                    "\nLeft Speed: %f"+
                                    "\nRight Speed: %f"+
                                    "\nWant to go to: %f"+
                                    "\nSo turning of: %f",
                                    CURRiteration,
//                                    currentDegree,
                                    leftCXFlow,
                                    rightCXFlow,
                                    CXnewHeading,
                                    CXtheta));
                        }
                    });

                } catch (Exception e) {
                    e.printStackTrace();
                }
                if(isHome(memory)) break;
                currentTime = (int) SystemClock.elapsedRealtime() - startTime;
            }

            try {
                sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            go(new double[]{0, 0});

            try {
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        debugTextView.setText(String.format("The PI Run is over! "+
                                "\nTo start again please return to the " +
                                "\nprevious page and click -Start-"));

                        // Show the "Put Robot Back to Food" Alert Dialog!
                        showDialog();
                    }
                });
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    };

    Runnable CXMBthread=new Runnable() {
        @Override
        public void run() {
            try {
                sleep(3000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            //Log.i(flowTag, "" + xPosOF + "," + yPosOF+","+orientation)

            Integer T_outbound = 40;
            Integer T_inbound = 30;//55;
            Integer T = T_inbound + T_outbound;

            SimpleMatrix tl2 = new SimpleMatrix(CX_MB.n_tl2, 1);
            tl2.set(0);
            SimpleMatrix cl1 = new SimpleMatrix(CX_MB.n_cl1, 1);
            cl1.set(0);
            SimpleMatrix tb1 = new SimpleMatrix(CX_MB.n_tb1, 1);
            tb1.set(0);
            SimpleMatrix memory = new SimpleMatrix(CX_MB.n_cpu4, 1);
            memory.set(.5);
            SimpleMatrix cpu4 = new SimpleMatrix(CX_MB.n_cpu4, 1);
            cpu4.set(0);
            SimpleMatrix cpu1 = new SimpleMatrix(CX_MB.n_cpu1, 1);
            cpu1.set(0);

            SimpleMatrix en = new SimpleMatrix(CX_MB.n_tb1, 1);
            cpu1.set(0);

            startTime = (int) SystemClock.elapsedRealtime();
            int t0 = (int) SystemClock.elapsedRealtime();
            int t = 0;
            CURRiteration = 0;
            int currentTime = (int)SystemClock.elapsedRealtime()-startTime;
            while (currentTime<T_outbound*1000) {
                CURRiteration++;
                try {
                    sleep(600);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                try {
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            debugTextView.setText(String.format("outbound " +
                                            "\nLeft Speed: %f"+
                                            "\nRight Speed: %f"+
                                            "\ncurrent iteration: %d"+
                                            "\ncurrent direction: %f",
                                    leftCXFlow,
                                    rightCXFlow,
                                    CURRiteration,
                                    currentDegree));
                        }
                    });
                } catch (Exception e) {
                    e.printStackTrace();
                }

                // OUTBOUND ROUTE
                t = (int) SystemClock.elapsedRealtime() - t0;

                String direction = "not sure";
                if(t>300) {
                    if (currentTime < 3500) {
                        go(new double[]{100, 100});
                        ANT_SPEED = 4.0;
                        direction = "straight";
                    } else if (currentTime < 8000) {
                        go(new double[]{10, 100});
                        ANT_SPEED = 2.0;
                        direction = "left";
                    } else if (currentTime < 11000) {
                        go(new double[]{100, 10});
                        ANT_SPEED = 2.0;
                        direction = "left";
                    } else if (currentTime < 25000) {
                        go(new double[]{100, 100});
                        ANT_SPEED = 4.;
                        direction = "straight";
                    } else {
                        go(new double[]{100, 10});
                        ANT_SPEED = 4.;
                        direction = "straight";
                    }
                    // based on odometry
                    //ANT_SPEED = ()

                    t0 = (int) SystemClock.elapsedRealtime();
                    Log.d(TAG, "facing " + currentDegree + ", going " + direction);
                    Log.d(TAG, printMemory(memory));
                }
                // updates in memory structure as we move which will allow return to nest
                //------   COMPASS UPDATE  -----
                tl2 = cxmb.tl2Output(Math.toRadians(currentDegree));
                cl1 = cxmb.cl1Output(tl2);
                tb1 = cxmb.tb1Output(cl1, tb1);

//                String tb1_str = "";
//                for(int i = 0; i < 8; i++){
//                    tb1_str += tb1.get(i, 0) + " ";
//                }
//                LogToFileUtils.write(tb1_str);

                // ------ DISPLACEMENT UPDATE -----
                memory = cxmb.cpu4Update(memory, tb1, ANT_SPEED);
                cpu4 = cxmb.cpu4Output(memory.copy());

                // ----- TURNING GENERATION ------
                cpu1 = cxmb.cpu1Output(tb1, cpu4, en, true);
                CXmotor = cxmb.motorOutput(cpu1);
                writeToFile(memory, leftCXFlow, rightCXFlow, "run_1");
                currentTime = (int) SystemClock.elapsedRealtime()-startTime;
            }

            // turn to offset pull, and allow network to redirect antbot
            go(new double[]{0, 0});
            try {
                sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            try{
                turnAround(45);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            currentTime = (int)SystemClock.elapsedRealtime()-t0;

            startTime = (int) SystemClock.elapsedRealtime();
            t0 = (int)SystemClock.elapsedRealtime();
            t = 0;

            stored_memory = memory.copy();

            Boolean algorithm_not_setted = true;

            while (algorithm_not_setted){
                if (images_access){
                    new_image = new LearnDirectionImages();
                    new_image.execute(processedDestImage, 7);
                    algorithm_not_setted = false;
                }
            }

            // INBOUND ROUTE
            while (currentTime < T_inbound*1000){
                CURRiteration++;
                try {
                    sleep(400);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                //------   COMPASS UPDATE  -----
                tl2 = cxmb.tl2Output(Math.toRadians(currentDegree));
                cl1 = cxmb.cl1Output(tl2);
                tb1 = cxmb.tb1Output(cl1, tb1);

                if (images_access) {
                    new_image = new LearnDirectionImages();
                    new_image.execute(processedDestImage, 5, tb1);
                }

                // ------ DISPLACEMENT UPDATE -----
                memory = cxmb.cpu4Update(memory, tb1, ANT_SPEED);
                cpu4 = cxmb.cpu4Output(memory.copy());

                // ----- TURNING GENERATION ------
                cpu1 = cxmb.cpu1Output(tb1, cpu4, en, true);
                CXmotor = cxmb.motorOutput(cpu1);
                writeToFile(memory, leftCXFlow, rightCXFlow, "run_1");

                // ----- ANTBOT DRIVING--------
                CXnewHeading = Math.toDegrees(Math.toRadians(currentDegree) - CXmotorChange * CXmotor);
                CXtheta = (CXnewHeading - currentDegree)%360;

                t = (int)SystemClock.elapsedRealtime() - t0;
                String direction = "not sure";
                if(t > 300){
                    // speed in dm/sec
                    if (CXtheta<-1.5){
                        go(new double[]{10, 100});
                        ANT_SPEED = 0.5;
                        direction = "left";
                    } else if (CXtheta>1.5){
                        go(new double[]{100, 10});
                        ANT_SPEED = 0.5;
                        direction = "right";
                    } else {
                        go(new double[]{100, 100});
                        ANT_SPEED = 4.;
                        direction = "straight";
                    }
                    Log.d(TAG, "facing " + currentDegree + ", going " + direction + "\nwant to go to " + CXnewHeading);
                    t0 = (int) SystemClock.elapsedRealtime();
                    Log.d(TAG, printMemory(memory));
                }

                try {
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            debugTextView.setText(String.format("inbound " +
                                            "\ncurrent iteration: %d"+
//                                    "\ncurrent direction: %f"+
                                            "\nLeft Speed: %f"+
                                            "\nRight Speed: %f"+
                                            "\nWant to go to: %f"+
                                            "\nSo turning of: %f",
                                    CURRiteration,
//                                    currentDegree,
                                    leftCXFlow,
                                    rightCXFlow,
                                    CXnewHeading,
                                    CXtheta));
                            //Toast.makeText(getApplication(), "famialarity:" + familarity, Toast.LENGTH_SHORT);
                        }
                    });

                } catch (Exception e) {
                    e.printStackTrace();
                }
                if(isHome(memory)) break;
                currentTime = (int) SystemClock.elapsedRealtime() - startTime;
            }

            try {
                sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            go(new double[]{0, 0});

            try {
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        debugTextView.setText(String.format("The PI Run is over! "+
                                "\nPlease Push the \"yes\" button when you are ready."));
                        // Show the "Put Robot Back to Food" Alert Dialog!
                        showDialog();
                    }
                });
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    };


    Runnable CXHolonomicThread=new Runnable() {
        @Override
        public void run() {
            try {
                sleep(3000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            CX_H_Pontin c = new CX_H_Pontin();
            Integer T_outbound = 40;            // outbound for 40 sec
            Integer T_inbound = 55;             // inbound for 55 sec
            Integer T = T_inbound + T_outbound; // total run

            SimpleMatrix tl2 = new SimpleMatrix(CX.n_tl2, 1);
            tl2.set(0);
            SimpleMatrix cl1 = new SimpleMatrix(CX.n_cl1, 1);
            cl1.set(0);
            SimpleMatrix tb1 = new SimpleMatrix(CX.n_tb1, 1);
            tb1.set(0);
            SimpleMatrix memory = new SimpleMatrix(CX.n_cpu4, 1);
            memory.set(.5);
            SimpleMatrix cpu4 = new SimpleMatrix(CX.n_cpu4, 1);
            cpu4.set(0);
            SimpleMatrix cpu1 = new SimpleMatrix(CX.n_cpu1, 1);
            cpu1.set(0);

            int startTime = (int) SystemClock.elapsedRealtime();
            int t0 = (int) SystemClock.elapsedRealtime();
            int t = 0;
            CURRiteration = 0;
            int currentTime = (int)SystemClock.elapsedRealtime()-startTime;

            while (currentTime<T_outbound*1000) {
                CURRiteration++;
                try {
                    sleep(400);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                // print output to screen - useful for DEBUG
                try {
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            debugTextView.setText(String.format(
                                    "outbound " +
                                    "\nLeft Speed: %f"+
                                    "\nRight Speed: %f" +
                                    "\nFrameRate: %f",
                                    leftCXFlow,
                                    rightCXFlow,
                                    frame_rate_cx
                                    )
                            );
                        }
                    });
                } catch (Exception e) {
                    e.printStackTrace();
                }

                // OUTBOUND ROUTE
                t = (int) SystemClock.elapsedRealtime() - t0;

                // hard coded outbound route
                String direction = "";
                if(t>300) {
                    if (currentTime < 3500) {
                        go(new double[]{100, 100});
                        ANT_SPEED = 4.0;
                        direction = "straight";
                    } else if (currentTime < 8000) {
                        go(new double[]{10, 100});
                        ANT_SPEED = 2.0;
                        direction = "left";
                    } else if (currentTime < 11000) {
                        go(new double[]{100, 10});
                        ANT_SPEED = 2.0;
                        direction = "left";
                    } else if (currentTime < 25000) {
                        go(new double[]{100, 100});
                        ANT_SPEED = 4.;
                        direction = "straight";
                    } else {
                        go(new double[]{100, 10});
                        ANT_SPEED = 4.;
                        direction = "straight";
                    }
                }
                t0 = (int) SystemClock.elapsedRealtime();
                Log.d(TAG, "facing " + currentDegree + ", going " + direction);
                Log.d(TAG, printMemory(memory));
                // updates in memory structure as we move which will allow return to nest
                //------   COMPASS UPDATE  -----
                tl2 = c.tl2Output(Math.toRadians(currentDegree));
                cl1 = c.cl1Output(tl2);
                tb1 = c.tb1Output(cl1, tb1);

                //---- Speed retrieval ------------
                SimpleMatrix flow = new SimpleMatrix(new double[][]{{leftCXFlow},{rightCXFlow}});
                SimpleMatrix tn1 = c.tn1Output(flow);
                SimpleMatrix tn2 = c.tn2Output(flow);

                // ------ DISPLACEMENT UPDATE -----
                memory = c.cpu4Update(memory, tb1, tn1, tn2);
                cpu4 = c.cpu4Output(memory.copy());

                // ----- TURNING GENERATION ------
                cpu1 = c.cpu1Output(tb1, cpu4);
                CXmotor = c.motorOutput(cpu1);
                currentTime = (int) SystemClock.elapsedRealtime()-startTime;
                writeToFile(memory, leftCXFlow, rightCXFlow, "run");
            }

            // offset turn, to allow network to redirect antbot
            go(new double[]{0, 0});
            try {
                sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            try{
                turnAround(45);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            currentTime = (int)SystemClock.elapsedRealtime()-t0;

            startTime = (int) SystemClock.elapsedRealtime();
            t0 = (int)SystemClock.elapsedRealtime();
            t = 0;

            // INBOUND ROUTE
            while (currentTime < T_inbound*1000){

                CURRiteration++;
                try {
                    sleep(400);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                //------   COMPASS UPDATE  -----
                tl2 = c.tl2Output(Math.toRadians(currentDegree));
                cl1 = c.cl1Output(tl2);
                tb1 = c.tb1Output(cl1, tb1);

                //---- Speed retrieval ------------
                SimpleMatrix flow = new SimpleMatrix(new double[][]{{1.},{1.}});
                SimpleMatrix tn1 = c.tn1Output(flow);
                SimpleMatrix tn2 = c.tn2Output(flow);

                // ------ DISPLACEMENT UPDATE -----
                memory = c.cpu4Update(memory, tb1, tn1, tn2);
                cpu4 = c.cpu4Output(memory.copy());

                // ----- TURNING GENERATION ------
                cpu1 = c.cpu1Output(tb1, cpu4);
                CXmotor = c.motorOutput(cpu1);
                writeToFile(memory, leftCXFlow, rightCXFlow,  "run");

                // ----- ANTBOT DRIVING--------

                // new direction based on memory
                CXnewHeading = Math.toDegrees(
                        (Math.toRadians(currentDegree) + CXmotor + Math.PI)
                                %(2.0 * Math.PI) - Math.PI
                );

                // given new direction, turn a litte right/left to approach it
                CXtheta = ((CXnewHeading - currentDegree) % 360);

                t = (int)SystemClock.elapsedRealtime() - t0;
                String direction = "";
                if(t > 300){
                    // speed in dm/sec
                    if (CXtheta < -1.5){
                        go(new double[]{10, 100});
                        ANT_SPEED = 0.5;
                        direction = "left";
                    } else if (CXtheta>1.5){
                        go(new double[]{100, 10});
                        ANT_SPEED = 0.5;
                        direction = "right";
                    } else {
                        go(new double[]{100, 100});
                        ANT_SPEED = 4.;
                        direction = "straight";
                    }
                    Log.d(TAG, "facing " + currentDegree + ", going " + direction + "\nwant to go to " + CXnewHeading);
                    t0 = (int) SystemClock.elapsedRealtime();
                    Log.d(TAG, printMemory(memory));
                }

                try {
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            float frame_rate;
                            if (currentFrameTime-prevFrameTime != 0){
                                frame_rate = 1/((currentFrameTime-prevFrameTime)*1000);
                            } else {
                                frame_rate = 0;
                            }
                            debugTextView.setText(String.format(
                                    "iutbound " +
                                            "\nLeft Speed: %f"+
                                            "\nRight Speed: %f" +
                                            "\nFrameRate: %f",
                                    leftCXFlow,
                                    rightCXFlow,
                                    frame_rate
                                    )
                            );
                        }
                    });

                } catch (Exception e) {
                    e.printStackTrace();
                }
                currentTime = (int) SystemClock.elapsedRealtime() - startTime;
//                if(isHome(memory)) break;

            }

            go(new double[]{0, 0});
            try {
                sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            go(new double[]{0, 0});

            try {
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        debugTextView.setText(String.format("The Run is over! "+
                                "\nTo start again please return to the " +
                                "\nprevious page and click -Start-"));
                    }
                });
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    };
    // Calculate the rms difference between two images
    private double rmsDifference(Mat current, Mat ref){
        int counter1 = 0;
        int counter3;
        double rootSum=0;
        int count =0;
        for (int cols = 0; cols < current.cols(); cols++) {
            counter3 = 0;
            for (int rows = 0; rows < current.rows(); rows++) {
                double xDiff = current.get(counter3,counter1)[0]-ref.get(counter3,counter1)[0];
                rootSum = rootSum + (Math.pow(xDiff,2));
                count++;
                counter3++;

            }
            counter1++;
        }
        return (Math.sqrt(rootSum/count));
    }

    //Rotate a 90x10 image (4 degrees intervals)
    private Mat rotateImage(Mat image, int theta){
        int counter1 = 0;
        int counter3;
        int colPosition;
        double[] rotPixel;
        if(theta<0){
            theta=theta+360;
        }
        theta = theta/4;
        Mat rotatedImage = Mat.zeros(image.size(),image.type());

        for (int phi = 0; phi < image.cols(); phi++) {
            counter3 = 0;
            for (int theta_tmp = 0; theta_tmp < image.rows(); theta_tmp++) {
                colPosition=counter1+theta;
                if(colPosition >=90){
                    colPosition=colPosition - 90;
                }
                rotPixel = image.get(counter3,colPosition);
                rotatedImage.put(counter3, counter1, rotPixel);
                counter3++;
            }
            counter1++;
        }
        broadcast.broadcastRotatedImage(rotatedImage);
        return rotatedImage;
    }

    private void computeSparseOpticFlow() {
        // first time around we initialize all
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
        // now we can compute the flow
        // left flow
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
            for (int i = 0; i < currentPointsToTrack.rows(); i++) {
                for (int j = 0; j < currentPointsToTrack.cols(); j++) {

                    // ------------------- compute left flow --------------------
                    //-----------------------------------------------------------
                    if (Math.abs(
                            mod((int) currentPointsToTrack.get(i, j)[0] + 12, 90)
                            - mod((int) prevPointsToTrack.get(i, j)[0] + 12, 90))
                            < 70){
                        current_left_flow_vector = new double[]{mod((int) currentPointsToTrack.get(i, j)[0] + 12, 90), (int) currentPointsToTrack.get(i, j)[1]};
                        previous_left_flow_vector = new double[]{mod((int) prevPointsToTrack.get(i, j)[0] + 12, 90), (int) prevPointsToTrack.get(i, j)[1]};

                        //   print arrows on debugFlowImage image - DEBUG
                        org.opencv.core.Point pt1_left = new org.opencv.core.Point(
                                previous_left_flow_vector[0], previous_left_flow_vector[1]);
                        org.opencv.core.Point pt2_left = new org.opencv.core.Point(
                                current_left_flow_vector[0], current_left_flow_vector[1]);
                        arrowedLine(debugFlowImage, pt1_left, pt2_left, new Scalar(0));
//                        //-----------------------------------------------------------

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
                    //-----------------------------------------------------------
                    if (Math.abs(
                            mod((int) currentPointsToTrack.get(i, j)[0] - 12, 90)
                            - mod((int) prevPointsToTrack.get(i, j)[0] - 12, 90))
                            < 70){
                        double[] current_right_flow_vector = {mod((int) currentPointsToTrack.get(i, j)[0] - 12, 90), (int) currentPointsToTrack.get(i, j)[1]};
                        double[] previous_right_flow_vector = {mod((int) prevPointsToTrack.get(i, j)[0] - 12, 90), (int) prevPointsToTrack.get(i, j)[1]};

                        //   print arrows on debugFlowImage image - DEBUG
                        org.opencv.core.Point pt1_right = new org.opencv.core.Point(
                                previous_right_flow_vector[0],
                                previous_right_flow_vector[1] + right_image_row);
                        org.opencv.core.Point pt2_right = new org.opencv.core.Point(
                                current_right_flow_vector[0],
                                current_right_flow_vector[1] + right_image_row);
                        arrowedLine(debugFlowImage, pt1_right, pt2_right, new Scalar(0));
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
            leftCXFlow = -1000*leftFlowSum/(totalpoints*(currentFrameTime-prevFrameTime));
            rightCXFlow = -1000*rightFlowSum/(totalpoints*(currentFrameTime-prevFrameTime));

            current_image.copyTo(previous_image);
            goodFeaturesToTrack(current_image, initialImage, 100, 0.5, 3);
            initialImage.convertTo(prevPointsToTrack, CvType.CV_32FC2);
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
               
                current_left_flow_vector = new double[]{mod((int) currentPointsToTrack.get(y, x)[0] + x + 12, 90),
                                                        mod((int) currentPointsToTrack.get(y, x)[1] + y, 10)};
                previous_left_flow_vector = new double[]{mod(x + 12, 90), y};
                if (Math.abs(previous_left_flow_vector[0] - current_left_flow_vector[0]) < 70) {
                    //   print arrows on debugFlowImage image - DEBUG
                    if (x + 12 == 20 || x + 12 == 70) {
                        org.opencv.core.Point pt1_left = new org.opencv.core.Point(
                                previous_left_flow_vector[0], previous_left_flow_vector[1]);
                        org.opencv.core.Point pt2_left = new org.opencv.core.Point(
                                current_left_flow_vector[0], current_left_flow_vector[1]);
                        arrowedLine(debugFlowImage, pt1_left, pt2_left, new Scalar(0));
                    }
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

                current_right_flow_vector = new double[] {mod((int) currentPointsToTrack.get(y, x)[0] + x - 12, 90),
                                                          mod((int) currentPointsToTrack.get(y, x)[1] + y, 10)};
                previous_right_flow_vector = new double[] {mod(x - 12, 90), y};

                if (previous_right_flow_vector[0] - current_right_flow_vector[0] < 70) {
                    //   print arrows on debugFlowImage image - DEBUG
                    if (x - 12 == 20 || x - 12 == 70) {
                        org.opencv.core.Point pt1_right = new org.opencv.core.Point(
                                previous_right_flow_vector[0],
                                previous_right_flow_vector[1] + right_image_row);
                        org.opencv.core.Point pt2_right = new org.opencv.core.Point(
                                current_right_flow_vector[0],
                                current_right_flow_vector[1] + right_image_row);
                        arrowedLine(debugFlowImage, pt1_right, pt2_right, new Scalar(0));
                    }
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

    // Drive forward
    public void moveForward(double dist){
        try{
            sleep(100);
        }catch(InterruptedException e){
            e.printStackTrace();
        }
        broadcast.broadcastDistance(dist);

    }

    public void go(double[] speeds){
        try{
            sleep(100);
        }catch(InterruptedException e){
            e.printStackTrace();
        }
        broadcast.broadcastMove(speeds);
    }

    // Turn by an angle
    public void turnAround(double theta) throws InterruptedException {
        try {
            sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        if(theta>180){
            theta=theta-360;
        }else if(theta<-180){
            theta=theta+360;
        }
        beginTurn=true;
        xTotal=0;
        if(theta<0){
            direction=2;
        }else if(theta>0){
            direction=1;
        }else{
            direction=0;
        }
        broadcast.broadcastAngle(theta);
        sleep(3000);
        broadcast.broadcastOpticalFlowAngle(xTotal);

    }

    private int mod(int x, int y)
    {
        int result = x % y;
        return result < 0 ? result + y : result;
    }

    private void writeToFile(SimpleMatrix memory, double left_speed, double right_speed, String filename) {

        File sd = new File(Environment.getExternalStorageDirectory(),"/DCIM/");
        filename += ".txt";
        File dest = new File(sd, filename);
        try {
            FileOutputStream stream = new FileOutputStream(dest, true);
            String logging_info = get_logging_data(memory, left_speed, right_speed);
            stream.write(logging_info.getBytes());
            stream.flush();
            stream.close();
        } catch (Exception e){
            e.printStackTrace();
        }
    }

    private String get_logging_data(SimpleMatrix memory, double left_speed, double right_speed) {
        int time = (int) SystemClock.elapsedRealtime();
        return  ""+myFormat(frame_rate_cx)+" || "+myFormat(CURRiteration)+" || "+
                ""+myFormat(currentDegree)+" || "+myFormat(time/1000)+" || "+
                ""+myFormat(left_speed)+"|"+ myFormat(right_speed)+" || "+
                ""+myFormat(memory.get(0))+","+myFormat(memory.get(8))+"|" +
                ""+myFormat(memory.get(7))+","+myFormat(memory.get(15))+"|" +
                ""+myFormat(memory.get(6))+","+myFormat(memory.get(14))+"|" +
                ""+myFormat(memory.get(5))+","+myFormat(memory.get(13))+"|" +
                ""+myFormat(memory.get(4))+","+myFormat(memory.get(12))+"|" +
                ""+myFormat(memory.get(3))+","+myFormat(memory.get(11))+"|" +
                ""+myFormat(memory.get(2))+","+myFormat(memory.get(10))+"|" +
                ""+myFormat(memory.get(1))+","+myFormat(memory.get(9))+"\n";
    }

    private String myFormat(float num){
        if (num< 0) return String.format("%02.4f", num);
        else return " "+String.format("%02.4f", num);
    }

    private String myFormat(double num){
        if (num< 0) return String.format("%02.4f", num);
        else return " "+String.format("%02.4f", num);
    }

    private String myformat(int num){
        if (num< 0) return String.format("%5d", num);
        else return " "+String.format("%5d", num);
    }


    private boolean isHome(SimpleMatrix memory){
        int time = (int) SystemClock.elapsedRealtime();
        int count = 0;
        for(int i=0; i< 16; i++){
            if (Math.abs(memory.get(i) - 0.5) < 0.01){
                count ++;
            }
        }
        return count>14 & time>45000;
    }


    static String printMemory(SimpleMatrix memory){
        return  "                "+memory.get(0) +"\n"+
                "                "+memory.get(8)+"\n"+
                "      "+memory.get(1)+"     "+memory.get(7)+"\n"+
                "      "+memory.get(9)+"     "+memory.get(15)+"\n"+
                memory.get(2)+"             "+memory.get(6)+"\n"+
                memory.get(10)+"             "+memory.get(14)+"\n"+
                "      "+memory.get(3)+"     "+memory.get(5)+"\n"+
                "      "+memory.get(11)+"     "+memory.get(13)+"\n"+
                "                "+memory.get(4)+"\n"+
                "                "+memory.get(12);
    }

    /**
     * start searching
     */

    Runnable startSearch=new Runnable() {
        @Override
        public void run() {
            for(;;){
                if(savedList){
                    savedList=false;
                    try {
                        sleep(15000);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    broadcast.broadcastStartSearch(true);
                }
            }

        }
    };

    public static int getMinIndex(double[] arr){
        double minNum = arr[0];
        int flag = 0;
        for(int i = 0; i < arr.length; i++){
            if(arr[i] < minNum){
                minNum = arr[i];
                flag = i;
            }
        }
        return flag;
    }

    Runnable startScanning = new Runnable() {
        @Override
        public void run() {

            try {
                sleep(3000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            int min_index = 0;

            while (true) {
                try {
                    turnAround(30.00);
                } catch (Exception e) {
                    e.printStackTrace();
                }

                Boolean image_not_accessed;

                String unfarmiliarity_distribution = "";

                for (int i = 0; i < 11; i++) {
                    image_not_accessed = true;
                    while (image_not_accessed){
                        if (images_access){
                            Mat matImage = processedDestImage;
                            //Since there is no possibility to send a 2D array or Mat file the image is transformed to a 1D int array
                            byte[] imageArray_tmp = new byte[matImage.height() * matImage.width()];
                            matImage.get(0, 0, imageArray_tmp);
                            int[] imageArray = new int[imageArray_tmp.length];
                            for (int n = 0; n < imageArray_tmp.length; n++) {
                                imageArray[n] = (int) imageArray_tmp[n] & 0xFF;
                            }
                            familarity_array[i] = model.calculateFamiliarity(imageArray);
                            unfarmiliarity_distribution += familarity_array[i] + ",";
                            image_not_accessed = false;
                        }}

                    if (i != 10) {
                        try {
                            turnAround(-6.00);
                        } catch (Exception e) {
                            e.printStackTrace();
                        }
                    }
                }

                LogToFileUtils.write(unfarmiliarity_distribution+'\n');

                min_index = getMinIndex(familarity_array);

                LogToFileUtils.write("Seleted index: " + min_index + "\n");

                try {
                    turnAround((10 - min_index) * 6.0);
                } catch (Exception e) {
                    e.printStackTrace();
                }

                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                moveForward(0.10);

                try {
                    sleep(2000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

        }
    };


    Runnable startKlinokinesis = new Runnable() {
        @Override
        public void run() {

            try {
                sleep(3000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            double current_familiarity;
            double maximum_unfamiliarity = model.calculateMaximumUnfamiliarity();
            LogToFileUtils.write("Maximum Unfamiliarity: " + maximum_unfamiliarity + "\n");
            double normalised_familiarity;

            // Tuning parameters
            double alpha = 80.0;
            double beta = 0.1;

            // Moving Decision
            double turn_angle;
            double step_size;

            int flag = 1;

            while(true) {
                if (images_access){
                    Mat matImage = processedDestImage;
                    //Since there is no possibility to send a 2D array or Mat file the image is transformed to a 1D int array
                    byte[] imageArray_tmp = new byte[matImage.height() * matImage.width()];
                    matImage.get(0, 0, imageArray_tmp);
                    int[] imageArray = new int[imageArray_tmp.length];
                    for (int n = 0; n < imageArray_tmp.length; n++) {
                        imageArray[n] = (int) imageArray_tmp[n] & 0xFF;
                    }

                    current_familiarity = model.calculateFamiliarity(imageArray);
                    LogToFileUtils.write("Current Familiarity: " + current_familiarity+ "\n");

                    normalised_familiarity = current_familiarity / maximum_unfamiliarity;
                    LogToFileUtils.write("Normalised Familiarity: " + normalised_familiarity + "\n");

                    turn_angle = alpha * normalised_familiarity;
                    LogToFileUtils.write("Turn Angle: " + turn_angle + "\n");

                    step_size = (1 - normalised_familiarity) * beta;
                    LogToFileUtils.write("Step Size: " + step_size + "\n");


                    try {
                        turnAround(flag * turn_angle);
                    } catch (Exception e) {
                        e.printStackTrace();
                    }

                    try {
                        sleep(500);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }

                    moveForward(step_size);

                    try {
                        sleep(2000);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }

                    flag = -flag;
                }
            }
        }
    };


    Runnable startCombiner = new Runnable() {
        @Override
        public void run() {

            try {
                sleep(3000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            SimpleMatrix tl2 = new SimpleMatrix(CX_MB.n_tl2, 1);
            tl2.set(0);
            SimpleMatrix cl1 = new SimpleMatrix(CX_MB.n_cl1, 1);
            cl1.set(0);
            SimpleMatrix tb1 = new SimpleMatrix(CX_MB.n_tb1, 1);
            tb1.set(0);
            SimpleMatrix memory = new SimpleMatrix(CX_MB.n_cpu4, 1);
            memory = stored_memory.copy();
            SimpleMatrix cpu4 = new SimpleMatrix(CX_MB.n_cpu4, 1);
            cpu4.set(0);
            SimpleMatrix cpu1 = new SimpleMatrix(CX_MB.n_cpu1, 1);
            cpu1.set(0);

            // Zhaoyu added this
            SimpleMatrix en = new SimpleMatrix(CX_MB.n_tb1, 1);
            en.set(0);

            int T_inbound = 30;

            startTime = (int) SystemClock.elapsedRealtime();
            int t0 = (int) SystemClock.elapsedRealtime();
            int t = 0;
            CURRiteration = 0;
            int currentTime = (int) SystemClock.elapsedRealtime() - t0;

            Boolean image_not_accessed;

            while (currentTime < T_inbound * 1000) {
                CURRiteration++;
                try {
                    sleep(400);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                image_not_accessed = true;
                while (image_not_accessed){
                    if (images_access){
                        Mat matImage = processedDestImage;
                        //Since there is no possibility to send a 2D array or Mat file the image is transformed to a 1D int array
                        byte[] imageArray_tmp = new byte[matImage.height() * matImage.width()];
                        matImage.get(0, 0, imageArray_tmp);
                        int[] imageArray = new int[imageArray_tmp.length];
                        for (int n = 0; n < imageArray_tmp.length; n++) {
                            imageArray[n] = (int) imageArray_tmp[n] & 0xFF;
                        }
                        en = cxmb.calculateFamiliarityDistribution(imageArray);
                        image_not_accessed = false;
                        LogToFileUtils.write("Image Updated");
                    }
                }

                //------   COMPASS UPDATE  -----
                tl2 = cxmb.tl2Output(Math.toRadians(currentDegree));
                cl1 = cxmb.cl1Output(tl2);
                tb1 = cxmb.tb1Output(cl1, tb1);

                // ------ DISPLACEMENT UPDATE -----
                memory = cxmb.cpu4Update(memory, tb1, ANT_SPEED);
                cpu4 = cxmb.cpu4Output(memory.copy());

                // ----- TURNING GENERATION ------
                cpu1 = cxmb.cpu1Output(tb1, cpu4, en, false);
                //cpu1 = cxmb.cpu1EnOutput(tb1, en);
                CXmotor = cxmb.motorOutput(cpu1);
                //writeToFile(memory, leftCXFlow, rightCXFlow, "run_1");

                // ----- ANTBOT DRIVING--------
                CXnewHeading = Math.toDegrees(Math.toRadians(currentDegree) - CXmotorChange * CXmotor);
                CXtheta = (CXnewHeading - currentDegree) % 360;

                t = (int) SystemClock.elapsedRealtime() - t0;
                String direction = "not sure";
                if (t > 300) {
                    // speed in dm/sec
                    if (CXtheta < -1.5) {
                        go(new double[]{10, 100});
                        ANT_SPEED = 0.5;
                        direction = "left";
                    } else if (CXtheta > 1.5) {
                        go(new double[]{100, 10});
                        ANT_SPEED = 0.5;
                        direction = "right";
                    } else {
                        go(new double[]{100, 100});
                        ANT_SPEED = 4.0;
                        direction = "straight";
                    }
                    Log.d(TAG, "facing " + currentDegree + ", going " + direction + "\nwant to go to " + CXnewHeading);
                    t0 = (int) SystemClock.elapsedRealtime();
                    Log.d(TAG, printMemory(memory));
                }

                try {
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            debugTextView.setText(String.format("inbound " +
                                            "\ncurrent iteration: %d" +
//                                    "\ncurrent direction: %f"+
                                            "\nLeft Speed: %f" +
                                            "\nRight Speed: %f" +
                                            "\nWant to go to: %f" +
                                            "\nSo turning of: %f",
                                    CURRiteration,
//                                    currentDegree,
                                    leftCXFlow,
                                    rightCXFlow,
                                    CXnewHeading,
                                    CXtheta));
                        }
                    });

                } catch (Exception e) {
                    e.printStackTrace();
                }
                if (isHome(memory)) break;
                currentTime = (int) SystemClock.elapsedRealtime() - startTime;
            }

            try {
                sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            go(new double[]{0, 0});

            try {
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        debugTextView.setText(String.format("The PI Run is over! " +
                                "\nTo start again please return to the " +
                                "\nprevious page and click -Start-"));
                    }
                });
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    };


    /**
     *
     * Start Homing
     */
    Runnable startHome = new Runnable() {
        @Override
        public void run() {
            for(;;){
                if(savedList){
                    savedList=false;
                    try {
                        sleep(15000);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    broadcast.broadcastStartHome(true);
                }
            }
                }
    };

    Runnable saveImagesRunDown =new Runnable() {
        @Override
        public void run() {
            try {
                sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            Mat temp=new Mat();
            for(int i=0;i<360;i++){
                temp=rotateFullImage(fullSnapShot,i);
            }
            Log.i(flowTag,"Saving Images Done");
            savedList=true;
        }
    };

    Runnable saveImagesTurnMinimum = new Runnable() {
        @Override
        public void run() {
            for(int n=0;n<4;n++){
                Log.i(flowTag,"start with list "+(n+1));
                Mat temp=new Mat();
                for(int i=0;i<360;i=i+4){
                    temp=rotateImage(imageToDisplay, i);
                }
                Log.i(flowTag,"Done with list "+(n+1));
                try {
                    sleep(15000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

            Log.i(flowTag,"Images saved");
            savedList=true;

        }
    };

    /**
     * The broadcast of the image takes place in a different thread so we have a proper running UI
     */
    private class BroadcastImage extends AsyncTask<Object, Integer, Boolean> {

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
            //Log.i(myTag, "Broadcasting*******");
            broadcast.broadcastImage(imageArray);
            return false;
        }
    }

    private class SaveImages extends AsyncTask<Object, Integer, Boolean> {

        @Override
        protected Boolean doInBackground(Object... transmission) {
            Mat matImage = (Mat) transmission[0];
            //Since there is no possibility to send a 2D array or Mat file the image is transformed to a 1D int array
            byte[] imageArray_tmp = new byte[matImage.height() * matImage.width()];
            matImage.get(0, 0, imageArray_tmp);
            int[] imageArray = new int[imageArray_tmp.length];
            String image_string="";
            for (int n = 0; n < imageArray_tmp.length; n++) {
                imageArray[n] = (int) imageArray_tmp[n] & 0xFF;
                image_string += imageArray[n]+ " ";
            }

            model.onNewImage(imageArray, (int) transmission[1]);
            // LogToFileUtils.write("IMAGE: " + image_string);
            // LogToFileUtils.write("REQUEST CODE: " + (int) transmission[1]);
            return false;
        }
    }

    private class LearnDirectionImages extends AsyncTask<Object, Integer, Boolean> {

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