package insectsrobotics.imagemaipulations.NavigationModules._Superclasses;

import android.widget.Toast;
//import android.content.res.Resources;
import android.os.Bundle;
import android.util.Log;
import android.widget.Toast;

import org.ejml.simple.SimpleMatrix;

import insectrobotics.broadcastlibrary.BroadcastValues;
import insectsrobotics.imagemaipulations.R;

/**
 * This Superclass should be extended by every navigation module, it manages the sending and receiving
 * of Data packets and
 */

public abstract class NavigationModules implements VisualNavigationInterface, BroadcastValues{

    private final String TAG = this.getClass().getSimpleName();
    Bundle mBundle = new Bundle();
    //Resources resources;
    Boolean learn_image_with_direction = false;

    //public NavigationModules(Resources res){resources = res;}
    public NavigationModules(){

    }

    public Bundle onNewImage(int[] imageArray, int REQUEST_CODE){
        Log.e(TAG,"onNewImage Called");
        Log.e(TAG, "Image Length = " + imageArray.length + " REQUEST_CODE = " + REQUEST_CODE);
        Log.e(TAG, "RequestCode = " + REQUEST_CODE);
        switch (REQUEST_CODE) {
            case CALCULATE_ERROR:
                getFamiliarity(imageArray);
                break;
            case SETUP_ALGORITHM:
                setupLearningAlgorithm(imageArray);
                break;
            case LEARN_IMAGE:
                learnImage(imageArray);
                break;
        }
        return mBundle;
    }

    @Override
    public void setupLearningAlgorithm(int[] image) {
        Log.d(TAG, "setupLearningAlgorithm called");

        //mBundle.putString(resources.getString(R.string.VABundleMethod), "SetupAlgorithm");
    }

    @Override
    public void learnImage(int[] image) {

        Log.d(TAG,"learnImage Called");

//
//        String method = mBundle.getString(resources.getString(R.string.VABundleMethod));
//        if (method != null) {
//            if (!method.equals("SetupAlgorithm")) {
//                mBundle.putString(resources.getString(R.string.VABundleMethod), "LearnImage");
//            }
//        } else {
//            mBundle.putString(resources.getString(R.string.VABundleMethod), "LearnImage");
//        }
    }

    @Override
    public double calculateFamiliarity(int[] image) {

        Log.d(TAG,"calculateFamiliarity Called");

        //mBundle.putString(resources.getString(R.string.VABundleMethod), "ErrorCalculation");

        return 0;
    }


    public void getFamiliarity(int[] image){
        Log.d(TAG,"getFamiliarity Called");
        double familiarity = calculateFamiliarity(image);
        Log.d(TAG,"Familiarity = " + familiarity);
        //mBundle.putDouble("Error", familiarity);

    }

    public void learnImageWithDirection(int[] image, SimpleMatrix tb1){
        Log.d(TAG,"learnImageWithDirection Called");
    }
}
