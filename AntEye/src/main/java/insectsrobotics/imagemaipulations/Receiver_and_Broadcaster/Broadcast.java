package insectsrobotics.imagemaipulations.Receiver_and_Broadcaster;


import android.content.Context;
import android.content.res.Resources;
import android.os.AsyncTask;
import android.os.Bundle;

import org.opencv.core.Mat;

import insectrobotics.broadcastlibrary.BroadcastValues;
import insectrobotics.broadcastlibrary.Broadcaster;

public class Broadcast extends Broadcaster implements BroadcastValues {

    Resources resources;

    public Broadcast(Context context, Resources res) {
        super(context);
        resources = res;
    }

    public void broadcastServerConnection(Bundle bundle){
        initiateBroadcast(SERVER_CONNECTION_BROADCAST);
        onNewDataToBroadcast(bundle);
        executeBroadcast();
    }

    public void broadcastImageForServer(Mat image){
        TranslateImage translateImage = new TranslateImage();
        translateImage.execute(image);
    }

    public void broadcastRotatedImage(Mat image){
        TranslateImageRotate translateImage = new TranslateImageRotate();
        translateImage.execute(image);
    }

    public void broadcastFlow(double theta){
        initiateBroadcast(FLOW_BROADCAST);
        onNewDataToBroadcast(theta);
        executeBroadcast();
    }

    public void broadcastCompleteFlag(boolean done){
        initiateBroadcast(COMPLETE_FLAG_BROADCAST);
        onNewDataToBroadcast(done);
        executeBroadcast();
    }

    public void broadcastStartHome(boolean start){
        initiateBroadcast(START_HOME_BROADCAST);
        onNewDataToBroadcast(start);
        executeBroadcast();
    }

    public void broadcastStartSearch(boolean start){
        initiateBroadcast(START_SEARCH_BROADCAST);
        onNewDataToBroadcast(start);
        executeBroadcast();
    }

    public void broadcastAngle(double theta){
        initiateBroadcast(ANGLE_BROADCAST);
        onNewDataToBroadcast(theta);
        executeBroadcast();
    }

    public void broadcastDistance(double distance){
        initiateBroadcast(DISTANCE_BROADCAST);
        onNewDataToBroadcast(distance);
        executeBroadcast();
    }

    public void broadcastMove(double[] speeds){
        initiateBroadcast(SPEED_BROADCAST);
        onNewDataToBroadcast(speeds);
        executeBroadcast();
    }

    public void broadcastOpticalFlowAngle(double angle){
        initiateBroadcast(OPTICAL_FLOW_ANGLE_BROADCAST);
        onNewDataToBroadcast(angle);
        executeBroadcast();
    }



    private class TranslateImage extends AsyncTask<Mat,String,byte[]> {

        @Override
        protected byte[] doInBackground(Mat... params) {
            Mat matImage = params[0];
            byte[] byteImage = new byte[matImage.cols()*matImage.rows()];
            matImage.get(0, 0, byteImage);
            return byteImage;
        }

        @Override
        protected void onPostExecute(byte[] bytes) {
            super.onPostExecute(bytes);
            initiateBroadcast(IMAGE_SERVER_BROADCAST);
            onNewDataToBroadcast(bytes);
            executeBroadcast();
        }
    }

    private class TranslateImageRotate extends AsyncTask<Mat,String,int[]> {
        @Override
        protected int[] doInBackground(Mat... params) {
            Mat matImage = params[0];
            byte[] byteImage = new byte[matImage.cols()*matImage.rows()];
            matImage.get(0, 0, byteImage);
            int[] imageArray=new int[byteImage.length];
            for(int n=0;n<byteImage.length;n++){
                imageArray[n]=(int) byteImage[n] & 0xFF;
            }
            return imageArray;
        }

        @Override
        protected void onPostExecute(int[] bytes) {
            super.onPostExecute(bytes);
            initiateBroadcast(IMAGE_ROTATE_BROADCAST);
            onNewDataToBroadcast(bytes);
            executeBroadcast();
        }
    }

    public void broadcastImage(int[] image){
        initiateBroadcast(IMAGE_BROADCAST);
        onNewDataToBroadcast(image);
        executeBroadcast();
    }

    // public void  broadcastModules(String visualModule, String integratorModule, String combinerModule)
    public void  broadcastModules(String visualModule){
        broadcastVisualModule(visualModule);
        //broadcastIntegratorModule(integratorModule);
        //broadcastCombinerModule(combinerModule);
    }
    private void broadcastVisualModule(String module){
        initiateBroadcast(VISUAL_MODULE_BROADCAST);
        onNewDataToBroadcast(module);
        executeBroadcast();
    }

    private void broadcastCombinerModule(String module){
        initiateBroadcast(COMBINER_MODULE_BROADCAST);
        onNewDataToBroadcast(module);
        executeBroadcast();
    }

    private void broadcastIntegratorModule(String module){
        initiateBroadcast(INTEGRATOR_MODULE_BROADCAST);
        onNewDataToBroadcast(module);
        executeBroadcast();
    }

    public void broadcastAlert(String alert){
        initiateBroadcast(ALERT_BROADCAST);
        onNewDataToBroadcast(alert);
        executeBroadcast();
    }
}
