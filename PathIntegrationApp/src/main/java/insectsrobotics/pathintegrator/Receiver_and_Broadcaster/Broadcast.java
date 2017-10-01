package insectsrobotics.pathintegrator.Receiver_and_Broadcaster;

import android.content.Context;
import android.content.res.Resources;

import insectrobotics.broadcastlibrary.BroadcastValues;
import insectrobotics.broadcastlibrary.Broadcaster;


public class Broadcast extends Broadcaster implements BroadcastValues {

    private static final String TAG = "OCVSample::Activity";

    Resources resources;

    public Broadcast(Context context, Resources res) {
        super(context);
        resources = res;
    }

    public void broadcastPathInformation(String path){
        initiateBroadcast(VECTOR_BROADCAST);
        onNewDataToBroadcast(path);
        executeBroadcast();
    }
    public void broadcastDistanceX(double distance){
        initiateBroadcast(POS_BROADCAST);
        onNewDataToBroadcast(distance);
        executeBroadcast();
    }

    public void broadcastDistanceY(double distance){
        initiateBroadcast(POSY_BROADCAST);
        onNewDataToBroadcast(distance);
        executeBroadcast();
    }

    public void broadcastAngle(double angle){
        initiateBroadcast(ORI_BROADCAST);
        onNewDataToBroadcast(angle);
        executeBroadcast();
    }

    public void broadcastPIModule(String module){
        initiateBroadcast(PI_BROADCAST);
        onNewDataToBroadcast(module);
        executeBroadcast();
    }

    public void broadcastDistanceData(double[] data){
        initiateBroadcast(DISTANCE_DATA_BROADCAST);
        onNewDataToBroadcast(data);
        executeBroadcast();
    }

    public void broadcastOrientationData(double[] data){
        initiateBroadcast(ORIENTATION_DATA_BROADCAST);
        onNewDataToBroadcast(data);
        executeBroadcast();
    }





}
