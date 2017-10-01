package insectsrobotics.antbotcombiner;


import android.content.Intent;
import android.os.IBinder;
import android.util.Log;

import insectrobotics.broadcastlibrary.BroadcastService;
import insectsrobotics.antbotcombiner.Receiver_and_Broadcast.Broadcast;
import insectsrobotics.antbotcombiner.Receiver_and_Broadcast.Receive;

import static java.lang.Thread.sleep;


public class CombinerBackgroundService extends BroadcastService {

    private final String TAG = this.getClass().getSimpleName();
    String flowTag = "flow";

    Broadcast broadcast;
    Receive receive;

    double xVector;
    double yVector;
    double orientation;

    String position;
    String errorString;
    String oldErrorString = "";
    boolean homing = false;
    boolean calcError = false;
    boolean VNReceived = false;
    boolean PIReceived = false;
    boolean visualAppAvailable = true;
    boolean moving = false;
    boolean turning = false;
    boolean requestData = false;
    boolean turningLeft = false;
    boolean turningRight = false;

    boolean searchingForMinimum = false;
    boolean turningToMinimum = false;

    double minimumError = Double.MAX_VALUE;
    double minimumOrientation;
    double oldError = -1;

    int calcCounter = 0;

    // PI data
    String modulePI;
    boolean stopPI =false;
    boolean startPI =false;
    double wheelsX;
    double wheelsY;
    double wheelsO;
    double opticalX;
    double opticalY;
    double opticalO;
    double fusedX;
    double fusedY;
    double fusedO;
    Thread homeRun;

    Receive.ReceiveListener listener = new Receive.ReceiveListener() {
        @Override
        public void onNewMessageFromSerialConnectionApp(Intent intent, String action) {
            switch (action){
                case TASK_EXECUTED:
                    if (searchingForMinimum){
                        Log.e(TAG, "Task Executed received");
                        requestData();
                    }
                    break;
                case HOMING_ROUTING:
                    startHoming();
                    break;
                case CALCULATE_ERROR_COMMAND:
                    if (!requestData) {
                        calcCounter = 0;
                        requestData();
                    }
                    break;
                case LEARN_IMAGE_COMMAND:
                    broadcast.requestNewVisualData(LEARN_IMAGE);
                    //broadcast.requestNewVisualData(SAVE_LEARN_IMAGE);
                    break;
                case TURN_TO_MINIMUM:
                    searchForMinimum();
                    break;
                case TYPE_OF_MOVEMENT:
                    if (intent.getStringExtra("MainData").equals("m")){
                        moving = true;
                        turning = false;
                        minimumError = Double.MAX_VALUE;
                    } else if (intent.getStringExtra("MainData").equals("t")){
                        moving = false;
                        turning = true;
                        minimumError = Double.MAX_VALUE;
                    } else if (intent.getStringExtra("MainData").equals("h")){
                        moving = false;
                        turning = false;
                        searchingForMinimum = false;
                        requestData = false;
                        turningToMinimum = false;
                    }
                default:
                    //Here could go some default stuff.
                    break;
            }
        }
        @Override
        public void onNewMessageFromCombinerApp(Intent intent, String action) {
        }
        @Override
        public void onNewMessageFromIntegratorApp(Intent intent, String action) {
            switch (action){
                case VECTOR:
                    String path = intent.getStringExtra("MainData");
                    parseVector(path);
                    if(homing) {
                        makeDecision();
                    }
                    break;
                case PI:
                    modulePI=intent.getStringExtra("MainData");
                    Log.i(flowTag,"Module PI "+modulePI);
                    homeRun=new Thread(returnHomeRunnable);
                    homeRun.start();
                    break;
                case DIST_DATA:
                    double[] vectorValues=intent.getDoubleArrayExtra("MainData");
                    wheelsX=vectorValues[0];
                    wheelsY=vectorValues[1];
                    opticalX=vectorValues[2];
                    opticalY=vectorValues[3];
                    fusedX=vectorValues[4];
                    fusedY=vectorValues[5];
                    Log.i(flowTag,"wheel x "+wheelsX+" wheel y "+wheelsY);
                    break;
                case ORI_DATA:
                    double[] orValues=intent.getDoubleArrayExtra("MainData");
                    wheelsO=orValues[0];
                    opticalO=orValues[1];
                    fusedO=orValues[2];
                    //Log.i(flowTag,"angle "+wheelsO);
                    break;
                default:
                    //Here could go some default stuff.
                    break;
            }
        }
        @Override
        public void onNewMessageFromVisualNavigationApp(Intent intent, String action) {
            switch (action){
                case ERROR:
                    String errors = "";
                    double error = intent.getBundleExtra("MainData").getDouble("Error");
                    double willshawError = error;
                    errors = errors + error;
                    error = intent.getBundleExtra("MainData").getDouble("PerfectMemoryError");
                    errorString = errors + "," + error;
                    if (!errorString.equals(oldErrorString) && !errorString.contains("0.0,0.0")) {
                        VNReceived = true;
                        oldErrorString = errorString;
                    }
                    Log.e(TAG,"Searching: " + searchingForMinimum + " requestData: " + requestData);
                    if (searchingForMinimum && requestData){
                        Log.e(TAG, "Error: " + willshawError);
                        if (willshawError > 175){
                            broadcast.broadcastDecision("t 2 m 0 n");
                        } else {
                            broadcast.broadcastDecision("h");
                            Log.e(TAG,"Minimum Reached");
                            searchingForMinimum = false;
                        }
                    }
                    /*
                    if (searchingForMinimum){
                        if (willshawError < minimumError){
                            minimumError = willshawError;
                            broadcast.requestNewIntegratorData();
                        }
                    }*/
                    requestData = false;
                    break;
                /*case STATUS_UPDATE:

                    visualAppAvailable = intent.getBooleanExtra("MainData", true);
                    if (visualAppAvailable){
                        requestData = false;
                    }
                    if (!requestData && calcCounter < 20){
                        requestData();
                    }

                    break;*/
                case TURN_DECISION:
                    double angle = intent.getDoubleExtra("MainData",0);
                    broadcast.broadcastDecision("t "+angle+" m 0 n");
                    break;
                case MOVEMENT_DECISION:
                    double distance = intent.getDoubleExtra("MainData",0);
                    broadcast.broadcastDecision("t 0 m "+distance+" n");
                    break;
                default:
                    //Here could go some default stuff.
                    break;
            }
        }
        @Override
        public void onNewMessageFromAntEyeApp(Intent intent, String action) {
            switch (action){
                case COMBINER_MODULE:
                    //TODO: Switch between Combiner Modules
                    break;
                case ALERT:
                    startPI=true;
                    break;
                case COMBINER_ANGLE:
                    double angle = intent.getDoubleExtra("MainData",0);
                    broadcast.broadcastDecision("t "+angle+" m 0 n");
                    break;
                case COMBINER_DISTANCE:
                    double distance = intent.getDoubleExtra("MainData",0);
                    broadcast.broadcastDecision("t 0 m "+distance+" n");
                    break;
                case COMBINER_SPEED:
                    double[] speeds = intent.getDoubleArrayExtra("MainData");
                    broadcast.broadcastDecision("g "+(int)speeds[0]+" "+(int)speeds[1]+" n");
                default:
                    //Here could go some default stuff.
                    break;

            }
        }
    };

    @Override
    public IBinder onBind(Intent intent) {
        Log.d(TAG, "onBind called");

        broadcast = new Broadcast(this);
        receive = new Receive(listener);
        registerReceiver(receive, receive.getIntentFilter());
        return null;
    }


    @Override
    public boolean onUnbind(Intent intent) {

        unregisterReceiver(receive);

        return super.onUnbind(intent);
    }

    private void startHoming(){
        homing = true;
        broadcast.requestNewIntegratorData();
    }

    private void parseVector(String path){
        String[] tokens = path.split(",");
        for (int n = 0; n < tokens.length; n++){
            if (tokens[n].contains("O")){
                tokens[n] = tokens[n].replace("O","");
                orientation = Double.parseDouble(tokens[n]);
                if (orientation >= 360){
                    orientation = orientation - 360;
                } else if (orientation <= -360){
                    orientation = orientation + 360;
                }
            } else if (tokens[n].contains("X")){
                tokens[n] = tokens[n].replace("X","");
                xVector = Double.parseDouble(tokens[n]);
            } else if (tokens[n].contains("Y")){
                tokens[n] = tokens[n].replace("Y","");
                yVector = Double.parseDouble(tokens[n]);
            }
        }
        if(orientation > 360){
            orientation = orientation - 360;
        } else if(orientation < -360){
            orientation = orientation + 360;
        }

        if (searchingForMinimum){
            minimumOrientation = orientation;
            Log.e(TAG, "MinOrientations: " + minimumOrientation);
        }
        if (turningToMinimum){
            Log.e(TAG,"Orientation: " + orientation + " MinOrientationEnd: " + minimumOrientation + "   " + minimumError);
            turnToMinimum();
            turningToMinimum = false;
        }

        Log.e(TAG, "O: " + orientation + " X: " + xVector + " Y: " + yVector);
    }

    private void makeDecision(){
        Log.e(TAG, "makeDecision called");
        double alpha, beta;
        int gamma;
        double distance;
        alpha = Math.toDegrees(Math.atan2(yVector, xVector));
        Log.e(TAG,"Alpha: " + alpha);
        beta = orientation;
        Log.e(TAG, "Beta: " + beta);
        gamma = (int)(-beta + alpha + 180);
        Log.e(TAG, "Gamma: " + gamma);
        distance = ((int)Math.sqrt((xVector*xVector)+(yVector*yVector)))/100;
        broadcast.broadcastDecision("t " + gamma + " m " + distance + " n");
    }

    private void requestData(){
        Log.d(TAG, "requestData called");
        broadcast.requestNewVisualData(CALCULATE_ERROR);
        //broadcast.requestNewVisualData(SAVE_IMAGE);
        requestData = true;
        //calcError = true;
        //calcCounter++;
    }


    private void searchForMinimum(){

        searchingForMinimum = true;
        minimumError = Double.MAX_VALUE;
        minimumOrientation = 0;
        broadcast.broadcastDecision("t 5 m 0 n");

    }

    private void turnToMinimum(){
        Log.e(TAG,"Decision: t " + (minimumOrientation-orientation) + " m 0 n");
        broadcast.broadcastDecision("t " + (minimumOrientation-orientation) + " m 0 n");
    }

    private void oscillationRoutine(){

    }

    private double calculateHomeDistance(double x, double y){
        return Math.sqrt(x*x + y*y)/100;
    }
    private double calculateHomeAngle(double x, double y){
        return Math.toDegrees(Math.atan2(y, x));
    }
    private int calculateTurnAngle(double turn, double or){
        return (int)(turn-or);
    }

    /**
     * Return Home by path integration data
     */

    private void returnHome(double x, double y, double theta) throws InterruptedException {
        double distance = calculateHomeDistance(x,y);
        double turn = calculateHomeAngle(x,y);
        int angle = calculateTurnAngle(turn,theta);
        angle = angle+180;
        if(angle<=-360){
            angle=angle+360;
        }else if(angle>=360){
            angle=angle-360;
        }
        broadcast.broadcastTurn(angle);
        sleep(5000);
        Log.i(flowTag,"x "+x+" y "+y+" o "+theta);
        for(double i = 0;i<distance;i=i+0.1){
            broadcast.broadcastDecision("t 0 m 0.05 n");
            sleep(1000);
        }
        stopPI=true;
    }

    Runnable returnHomeRunnable=new Runnable() {
        @Override
        public void run() {
            for(;;){
                if(startPI){
                    double x=0;
                    double y=0;
                    double o=0;
                    if(modulePI.equals(ENCODERS)){
                        x=wheelsX;
                        y=wheelsY;
                        o=wheelsO;
                    }else if(modulePI.equals(OPTICAL_FLOW)){
                        x=opticalX;
                        y=opticalY;
                        o=opticalO;
                    }else if (modulePI.equals(FUSED_SENSORS)){
                        x=fusedX;
                        y=fusedY;
                        o=fusedO;
                    }
                    try {
                        sleep(4000);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    try {
                        returnHome(x,y,o);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
                if(stopPI) return;
            }
        }
    };
}
