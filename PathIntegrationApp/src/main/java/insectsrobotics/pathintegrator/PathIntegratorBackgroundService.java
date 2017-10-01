package insectsrobotics.pathintegrator;


import android.content.Intent;
import android.content.IntentFilter;
import android.os.IBinder;
import android.support.annotation.Nullable;
import android.util.Log;

import insectrobotics.broadcastlibrary.BroadcastService;
import insectsrobotics.pathintegrator.Receiver_and_Broadcaster.Broadcast;
import insectsrobotics.pathintegrator.Receiver_and_Broadcaster.Receive;


public class PathIntegratorBackgroundService extends BroadcastService {

    String TAG = this.getClass().getSimpleName();
    String moduleSelection = "";

    double[] distanceData = new double[6];
    double[] orientantationData = new double[3];

    // Wheel encoders
    double orientation;
    double xVector;
    double yVector;

    //Optical flow readings
    double orientationOpticalFlow;
    double xVectorOpticalFlow;
    double yVectorOpticalFlow;

    // Data Fusion
    double orientationFusion;
    double xVectorFusion;
    double yVectorFusion;
    double predMean=0;
    double predVar=0;
    double fusedPI =0;
    double fusedVar=0;
    double meanPI=0;
    double varPI=6.25;
    double meanOF=0;
    double varOF=1.44;
    double prevPI=0;
    double prevOF=0;

    Broadcast broadcast;
    Receive receive;

    Receive.ReceiveListener listener = new Receive.ReceiveListener() {
        @Override
        public void onNewMessageFromSerialConnectionApp(Intent intent, String action) {
            switch(action) {
                case WHEEL_ENCODER:
                    parseData(intent.getStringExtra("MainData"));
                    break;
                default:
                    //Do some Default stuff here if wanted.
                    break;
            }
        }

        @Override
        public void onNewMessageFromCombinerApp(Intent intent, String action) {
            switch(action) {
                case REQUEST_PI_DATA:
                    String path = "O" + String.format("%.2f",orientation) +
                            ",X" + String.format("%.2f",xVector) +
                            ",Y" + String.format("%.2f",yVector);
                    Log.e(TAG,path);
                    broadcast.broadcastPathInformation(path);
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
                case INTEGRATOR_FLOW:
                    updateOpticalFlow(intent.getDoubleExtra("MainData",orientationOpticalFlow));
                    updateFusion();
                    break;
                case INTEGRATOR_MODULE:
                    moduleSelection=intent.getStringExtra("MainData");
                    broadcast.broadcastPIModule(moduleSelection);
            }

        }
    };


    @Nullable
    @Override
    public IBinder onBind(Intent intent) {
        Log.e("Service", "onBindCalled");
        broadcast = new Broadcast(this, getResources());
        receive = new Receive(listener);
        IntentFilter intentFilter = receive.getIntentFilter();
        registerReceiver(receive, intentFilter);
        return null;
    }


    @Override
    public boolean onUnbind(Intent intent) {
        unregisterReceiver(receive);
        return super.onUnbind(intent);
    }


    private void parseData(String data){
        String[] dataArray = data.split(" ");
        if(dataArray[0].equals("t")){
            orientation = orientation + Double.parseDouble(dataArray[1]);
            Log.e(TAG,"Orientation: " + orientation);
            if (orientation >= 360){
                orientation = orientation - 360;
            } else if(orientation <= -360){
                orientation = orientation + 360;
            }

            Log.e("IntegratorService", "Orientation: " + orientation);
        } else if(dataArray[0].equals("m") || dataArray[0].equals("g")){
            xVector = xVector + (Double.parseDouble(dataArray[1])*(Math.cos(Math.toRadians(orientation))))*1.8; // (*1.8)-Compansation for wheel encoders readings when moving by 5cm intervals, which is the norm in this project
            yVector = yVector + (Double.parseDouble(dataArray[1])*(Math.sin(Math.toRadians(orientation))))*1.8;
            xVectorOpticalFlow = xVectorOpticalFlow + (Double.parseDouble(dataArray[1])*(Math.cos(Math.toRadians(orientationOpticalFlow))))*1.8;
            yVectorOpticalFlow = yVectorOpticalFlow + (Double.parseDouble(dataArray[1])*(Math.sin(Math.toRadians(orientationOpticalFlow))))*1.8;
            xVectorFusion = xVectorFusion + (Double.parseDouble(dataArray[1])*(Math.cos(Math.toRadians(orientationFusion))))*1.8;
            yVectorFusion = yVectorFusion + (Double.parseDouble(dataArray[1])*(Math.sin(Math.toRadians(orientationFusion))))*1.8;

            distanceData= new double[]{xVector, yVector, xVectorOpticalFlow, yVectorOpticalFlow, xVectorFusion, yVectorFusion};
            broadcast.broadcastDistanceData(distanceData);

            Log.e("IntegratorService", "X: " + xVector + " Y: " + yVector);
        }
    }

    private void updateOpticalFlow(double data){
        orientationOpticalFlow=orientationOpticalFlow+data;
        if (orientationOpticalFlow >= 360){
            orientationOpticalFlow = orientationOpticalFlow - 360;
        } else if(orientationOpticalFlow <= -360){
            orientationOpticalFlow = orientationOpticalFlow + 360;
        }
    }


    private void updateFusion(){
        meanPI=orientation-prevPI;
        if(meanPI<0){
            meanPI=meanPI+360;
        }
        if(orientationFusion<0){
            orientationFusion=orientationFusion+360;
        }
        predMean=predict(fusedVar,orientationFusion,varPI,meanPI)[1];
        predVar=predict(fusedVar,orientationFusion,varPI,meanPI)[0];

        orientationFusion=correct(predVar,predMean,varOF,orientationOpticalFlow)[1];
        fusedVar=correct(predVar,predMean,varOF,orientationOpticalFlow)[0];
        prevPI=orientation;
        if(orientationFusion>360){
            orientationFusion=orientationFusion-360;
        }else if(orientationFusion<-360){
            orientationFusion=orientationFusion+360;
        }
        orientantationData= new double[]{orientation, orientationOpticalFlow, orientationFusion};
        broadcast.broadcastOrientationData(orientantationData);

    }

    private double[] predict(double var, double mean, double dVar, double dMean){
        double[] newValue = new double[2];
        newValue[0] = var + dVar;
        newValue[1] = mean + dMean;
        return newValue;

    }

    private double[] correct(double var, double mean, double sVar,  double sMean){
        double[] newValue = new double[2];
        newValue[0] = 1/((1/var) + (1/sVar));
        newValue[1] = (sVar*mean + var*sMean)/(sVar+var);
        return newValue;
    }
}
