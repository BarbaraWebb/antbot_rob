package insectsrobotics.visualnavigationapp;

import android.content.Intent;
import android.content.IntentFilter;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.IBinder;
import android.support.annotation.Nullable;
import android.util.Log;
import android.widget.Toast;

import java.util.HashMap;

import insectrobotics.broadcastlibrary.BroadcastService;
import insectsrobotics.visualnavigationapp.NavigationModules.PerfectMemoryModule;
import insectsrobotics.visualnavigationapp.NavigationModules.WillshawModule;
import insectsrobotics.visualnavigationapp.Receiver_and_Broadcaster.Broadcast;
import insectsrobotics.visualnavigationapp.Receiver_and_Broadcaster.Receive;

import static java.lang.Thread.sleep;


public class BackgroundService extends BroadcastService {

    private final String TAG = this.getClass().getSimpleName();
    String myTag = "Canny";
    String flowTag="flow";

    boolean firstCall = true;
    boolean errorRequest = false;
    boolean imageReceived = true;
    int REQUEST_CODE;

    String moduleName;
    WillshawModule willshawModule;
    PerfectMemoryModule perfectMemoryModule;

    String saveToFileString = "";

    AsyncTask<Object, Void, Bundle> myTask;

    FileManager fileManager;

    Broadcast broadcast;
    Receive receive;

    Bundle mBundle = new Bundle();

    //Visual Homing Variables
    boolean start=false;
    boolean nextList=false;
    String homingModule="";
    int[] comparisonImage;
    int[] currentImage;

    //tem
    double orient=60;
    double xV=45;
    double yV=40;


    HashMap<Integer, int[]>savedImageList1=new HashMap<>();
    int count=0;
    double rmsThreshold=20;
    boolean stopThread=false;
    Thread runDownThread;
    Thread turnToMinimumThread;
    Thread systematicSearchThread;

    Receive.ReceiveListener listener = new Receive.ReceiveListener() {
        @Override
        public void onNewMessageFromSerialConnectionApp(Intent intent, String action) {
            Log.d(TAG,"NewMessageFromSerial");
            switch (action){
                case RESET_LEARNING_COMMAND:
                    //TODO: Reset networks
                    break;
                case SEND_STRING:
                    saveToFileString = intent.getStringExtra("MainData") + "#";
                    Log.e(TAG,saveToFileString);
                    break;
                default:
                    //Some Default Code
                    break;
            }
        }

        @Override
        public void onNewMessageFromCombinerApp(Intent intent, String action) {
            switch (action){
                case REQUEST_VN_DATA:
                    REQUEST_CODE = intent.getIntExtra("MainData", -1);
                    Log.d(TAG,"REQUESTCODE onNewCombinerMSg: " + REQUEST_CODE);
                    if (REQUEST_CODE != -1) {
                        errorRequest = true;
                        if (firstCall) {
                            imageReceived = false;
                        }
                    }
                    if (REQUEST_CODE == CALCULATE_ERROR){
                        if (mBundle != null) {
                            //broadcast.broadcastFamiliarityError(mBundle);
                        }
                    }
                    break;
                default:
                    //Some Default Code if you want to.
                    break;
            }
        }
        @Override
        public void onNewMessageFromIntegratorApp(Intent intent, String action) {
        }
        @Override
        public void onNewMessageFromVisualNavigationApp(Intent intent, String action) {
        }

        @Override
        public void onNewMessageFromAntEyeApp(Intent intent, String action) {
            switch (action){
                case VISUAL_MODULE:
                    moduleName = intent.getStringExtra("MainData");
                    //homingModule=moduleName;
                    homingModule=TURN_TO_MIN;
                    Log.d(TAG, "visualModule: " + moduleName);
                    switch (moduleName){
                        case WILLSHAW:
                            willshawModule = new WillshawModule(getResources());
                            perfectMemoryModule = new PerfectMemoryModule(getResources());
                            break;
                        case PERFECT_MEMORY:
                            perfectMemoryModule = new PerfectMemoryModule(getResources());
                    }
                    break;
                case IMAGE:
                    int[] imageArray = intent.getIntArrayExtra("MainData");
                    currentImage=imageArray.clone();
                    break;
                case IMAGE_ROTATE:
                    if(homingModule.equals(RUNDOWN) || homingModule.equals(SYSTEMATIC_SEARCH) || homingModule.equals(TURN_TO_MIN)){
                        savedImageList1.put(count, intent.getIntArrayExtra("MainData"));
                        count++;
                        Log.i(flowTag,""+count);
                    }
                    break;
                case START_HOME:
                    if(homingModule.equals(RUNDOWN)){
                        runDownThread=new Thread(runDown);
                        runDownThread.start();
                    }else if(homingModule.equals(TURN_TO_MIN)){
                        turnToMinimumThread=new Thread(turnToMinimum);
                        turnToMinimumThread.start();
                    }
                    break;
                case START_SEARCH:
                    systematicSearchThread=new Thread(systematicSearch);
                    systematicSearchThread.start();
                    break;
            }
        }
    };



    @Nullable
    @Override
    public IBinder onBind(Intent intent) {
        Log.e(TAG, "onBindCalled");

        broadcast = new Broadcast(this,getResources());
        receive = new Receive(listener);
        Log.i(myTag,"This is binded");
        IntentFilter intentFilter = receive.getIntentFilter();
        registerReceiver(receive, intentFilter);

        //fileManager = new FileManager();
        //int numberOfImages = fileManager.loadNumberOfImages();
        //Log.e(TAG, "number of images: " + numberOfImages);
        //broadcast.broadcastNumberOfImages(numberOfImages);
        Toast.makeText(getApplicationContext(), "VisualNavigationService Connected", Toast.LENGTH_SHORT).show();
        return null;
    }

    @Override
    public boolean onUnbind(Intent intent) {
        stopThread=true;
        Log.d("onUnbind", "Unbind called");
        Toast.makeText(getApplicationContext(), "VisualNavigationService Disconnected", Toast.LENGTH_SHORT).show();
        unregisterReceiver(receive);
        return super.onUnbind(intent);
    }



    private void calculateError(int[] imageArray){
        Log.d(TAG,"calculateError called, RequestCode: " + REQUEST_CODE);
        myTask = new BackgroundCalculations();
        switch (REQUEST_CODE){
            case LOAD_LEARNED_IMAGE:
                imageArray = fileManager.loadLearnedImage();
                REQUEST_CODE = LEARN_IMAGE;
                break;
            case LOAD_IMAGE:
                imageArray = fileManager.loadImage();
                REQUEST_CODE = CALCULATE_ERROR;
                break;
            case SAVE_LEARN_IMAGE:
                fileManager.saveLearnedImage(saveToFileString, imageArray);
                break;
            case SAVE_IMAGE:
                fileManager.saveImage(saveToFileString, imageArray);
                broadcast.broadcastStatusUpdate(true);
                break;
            default:
                //Some Default stuff if wanted.
                break;
        }
        switch (REQUEST_CODE){
            case CALCULATE_ERROR:
                switch(moduleName){
                    case WILLSHAW:
                        if (willshawModule != null){
                            myTask.execute(imageArray, REQUEST_CODE);
                        }
                        break;
                    case PERFECT_MEMORY:
                        if (perfectMemoryModule != null){
                            myTask.execute(imageArray, REQUEST_CODE);
                        }
                        break;
                    default:
                        //Some default stuff or NoModule selection here.
                }
                break;
            case LEARN_IMAGE:
                learning = true;
                errorRequest = false;
                if (firstCall){
                    REQUEST_CODE = SETUP_ALGORITHM;
                    firstCall = false;
                }
                switch(moduleName){
                    case WILLSHAW:
                        if (willshawModule != null){
                            Log.e(TAG, WILLSHAW + " RequestCode: " + REQUEST_CODE);
                            myTask.execute(imageArray, REQUEST_CODE);
                        }
                        break;
                    case PERFECT_MEMORY:
                        if (perfectMemoryModule != null){
                            myTask.execute(imageArray, REQUEST_CODE);
                        }
                        break;
                    default:
                        //Some default stuff or NoModule selection here.
                }
                break;
            default:
                Log.d(TAG,"Default called");
                errorRequest = false;

        }
        if (REQUEST_CODE != SAVE_IMAGE && REQUEST_CODE != SAVE_LEARN_IMAGE) {
            REQUEST_CODE = CALCULATE_ERROR;
        } else {
            REQUEST_CODE = -1;
        }
    }

    /**
     * runDown Thread
     */
    Runnable runDown=new Runnable() {
        @Override
        public void run() {
            try {
                sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            double previous=99999;
            double currentImageDiff=99999;
            comparisonImage=savedImageList1.get(0).clone();
            int[] temp;
            double[] lowestRms=new double[3];
            for(;;){

                    moveForward(0.05);
                    try {
                        sleep(2000);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    currentImageDiff=rmsDifference(currentImage,comparisonImage);
                    Log.i(flowTag,"rms "+currentImageDiff);
                    if(currentImageDiff<=rmsThreshold){
                        return;
                    }
                    if(currentImageDiff>previous){
                        turnAround(90);
                        try {
                            sleep(3000);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }

                        lowestRms=lowestRMS();
                        currentImageDiff=lowestRms[0];
                        Log.i(flowTag,"angle: "+lowestRms[1]);
                        if(currentImageDiff<=rmsThreshold){
                            Log.i(flowTag,"rms "+currentImageDiff);
                            return;
                        }

                    }
                    previous=currentImageDiff;



                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                if(stopThread) return;
            }
        }
    };

    /*
    Turn to minimum Thread
     */
    Runnable turnToMinimum = new Runnable() {
        @Override
        public void run() {
            //Toast.makeText(getApplicationContext(),"")
//            boolean firstTime=true;
//            for(;;){
//                try {
//                    sleep(3000);
//                } catch (InterruptedException e) {
//                    e.printStackTrace();
//                }
//                double[] lowRms=lowestRMS2();
//                double angleTurn=lowRms[1];
//                double rmsValue=lowRms[0];
//                //Log.i(flowTag,"angle to turn "+angleTurn+" rmsValue "+rmsValue+"List no "+lowRms[2]);
//                if(firstTime){
//                    firstTime=false;
//                    if(lowRms[2]==2){
//                        angleTurn=angleTurn+180;
//                    }else if(lowRms[2]==3){
//                        angleTurn=angleTurn+90;
//                    }else if(lowRms[2]==4){
//                        angleTurn=angleTurn+270;
//                    }
//                }
//                if(rmsValue<=rmsThreshold){
//                    Log.i(flowTag,"rms "+rmsValue);
//                    return;
//                }
//
//                turnAround(angleTurn);
//                try {
//                    sleep(3000);
//                } catch (InterruptedException e) {
//                    e.printStackTrace();
//                }
//                moveForward(0.05);
//                Log.i(flowTag,""+xV+","+yV+","+rmsValue);
//                if(stopThread)return;
//            }

        }
    };


    /*
    Systematic search thread
     */
    Runnable systematicSearch=new Runnable() {
        @Override
        public void run() {
            try {
                sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            double m=0.05;
            double lowestRms;
            int increaseDiameter=0;
            for(;;){
                    for(double i=0;i<m;i+=0.05){
                        lowestRms=lowestRMS()[0];
                        try {
                            sleep(3000);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                        Log.i(flowTag,"rms "+lowestRms);
                        if(lowestRms<=rmsThreshold){
                            return;
                        }

                        moveForward(0.05);
                        try {
                            sleep(1000);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    }
                    turnAround(90);
                try {
                    sleep(2000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                increaseDiameter++;
                    if(increaseDiameter==2){
                        m+=0.05;
                        increaseDiameter=0;
                    }

                if(stopThread) return;
            }
        }
    };




    // Calculate the rms difference between two images
    private double rmsDifference(int[] current, int[] ref){
        double rootSum=0;
        int count =0;
        for (int cols = 0; cols < current.length; cols++) {
            double xDiff = current[cols] - ref[cols];
            rootSum = rootSum + (Math.pow(xDiff, 2));
            count++;
        }
        return (Math.sqrt(rootSum/count));
    }

    // Lowest RMS difference and angle to turn
    private double[] lowestRMS (){
        double[] lowestData;
        int[] temp = currentImage.clone();
        int counter=0;
        int angle=0;
        int listNumber=1;
        int inList =1;
        int toCompare=0;
        double lowRms=9999;
        for(int i=0; i<savedImageList1.size(); i++){
            if(counter==360){
                counter=0;
                listNumber++;
            }
            double currentRms=rmsDifference(temp,savedImageList1.get(i));
            if(currentRms<lowRms){
                lowRms=currentRms;
                angle=counter;
                inList=listNumber;
                toCompare=i;
            }
            counter++;
        }
        comparisonImage=savedImageList1.get(toCompare).clone();
        lowestData= new double[]{lowRms, angle, inList};

        return lowestData;

    }

    // Lowest RMS difference and angle to turn
    private double[] lowestRMS2 (){
        double[] lowestData;
        int[] temp = currentImage.clone();
        int counter=0;
        int angle=0;
        int listNumber=1;
        int inList =1;
        int toCompare=0;
        double lowRms=9999;
        for(int i=0; i<savedImageList1.size(); i++){
            if(counter==90){
                counter=0;
                listNumber++;
            }
            double currentRms=rmsDifference(temp,savedImageList1.get(i));
            if(currentRms<lowRms){
                lowRms=currentRms;
                angle=counter*4;
                inList=listNumber;
                toCompare=i;
            }
            counter++;
        }
        comparisonImage=savedImageList1.get(toCompare).clone();
        lowestData= new double[]{lowRms, angle, inList};

        return lowestData;

    }

    private void moveForward(double distance){
        broadcast.broadcastMovementDecision(distance);

    }

    private void turnAround(double theta){
        if(theta>180){
            theta=theta-360;
        }else if(theta<-180){
            theta=theta+360;
        }
        broadcast.broadcastTurnDecision(theta);
        orient=orient+theta;
    }

    boolean learning = false;

    public class BackgroundCalculations extends AsyncTask<Object, Void, Bundle> {

        String TAG = this.getClass().getSimpleName();

        public BackgroundCalculations(){

        }

        @Override
        protected void onPreExecute() {
            if (learning) {
                broadcast.broadcastStatusUpdate(false);
                learning = true;
            }
        }


        @Override
        protected Bundle doInBackground(Object... params) {

            String tmp;
            tmp = moduleName;
            String moduleName;
            moduleName = tmp;
            Log.d(TAG,"ModuleName: " + moduleName);
            mBundle = new Bundle();
            Bundle secBundle;

            if (moduleName.isEmpty()) {
                moduleName = NO_MODULE;
            }
            Log.e("doInBackground123", moduleName);

            switch (moduleName) {
                case WILLSHAW:
                    Log.d(TAG, WILLSHAW + " Called");
                    mBundle = willshawModule.onNewImage((int[]) params[0], (int) params[1]);
                    //secBundle = perfectMemoryModule.onNewImage((int[]) params[0], (int) params[1]);
                    //double error = secBundle.getDouble(getResources().getString(R.string.VABundleError));
                    mBundle.putDouble("PerfectMemoryError", 0.0);
                    break;
                case PERFECT_MEMORY:
                    mBundle = perfectMemoryModule.onNewImage((int[]) params[0], (int) params[1]);
                    break;
                default:
                    mBundle.clear();
                    break;
            }

            return mBundle;
        }

        @Override
        protected void onPostExecute(Bundle bundle) {
            Log.d(TAG, "onPostExecute Called: errorRequest: " + errorRequest);
            imageReceived = false;
            if (learning){
                Log.e(TAG,"About to make Status Update");
                broadcast.broadcastStatusUpdate(true);
                learning = false;
            }
            if (mBundle != null) {
                broadcast.broadcastFamiliarityError(mBundle);
            }
            if (errorRequest) {
                //broadcast.broadcastFamiliarityError(mBundle);
                Log.d(TAG,"postExecute called");
                errorRequest = false;
            }
        }
    }
}





