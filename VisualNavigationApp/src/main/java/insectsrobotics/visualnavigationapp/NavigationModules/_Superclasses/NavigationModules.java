package insectsrobotics.visualnavigationapp.NavigationModules._Superclasses;


import android.content.res.Resources;
import android.os.Bundle;
import android.util.Log;

import insectrobotics.broadcastlibrary.BroadcastValues;
import insectsrobotics.visualnavigationapp.R;

/**
 * This Superclass should be extended by every navigation module, it manages the sending and receiving
 * of Data packets and
 */

public abstract class NavigationModules implements VisualNavigationInterface, BroadcastValues{

    private final String TAG = this.getClass().getSimpleName();
    Bundle mBundle = new Bundle();
    Resources resources;

    public NavigationModules(Resources res){
        resources = res;
    }

    public Bundle onNewImage(int[] imageArray, int REQUEST_CODE){
        Log.e(TAG,"onNewImage Called");
        Log.e(TAG, "Image Length = " + imageArray.length + " REQUEST_CODE = " + REQUEST_CODE);
        Log.e(TAG, "RequestCode = " + REQUEST_CODE);
        if (REQUEST_CODE == CALCULATE_ERROR) {
            getFamiliarity(imageArray);
        }
        else if (REQUEST_CODE == LEARN_IMAGE || REQUEST_CODE == SETUP_ALGORITHM){
            if (REQUEST_CODE == SETUP_ALGORITHM){
                setupLearningAlgorithm(imageArray);
            }
            learnImage(imageArray);
        }
        else {
            mBundle.putString(resources.getString(R.string.VABundleMethod), "NoMethodCalled");
        }

        return mBundle;

    }

    @Override
    public void setupLearningAlgorithm(int[] image) {
        Log.d(TAG, "setupLearningAlgorithm called");

        mBundle.putString(resources.getString(R.string.VABundleMethod), "SetupAlgorithm");
    }

    @Override
    public void learnImage(int[] image) {

        Log.d(TAG,"learnImage Called");


        String method = mBundle.getString(resources.getString(R.string.VABundleMethod));
        if (method != null) {
            if (!method.equals("SetupAlgorithm")) {
                mBundle.putString(resources.getString(R.string.VABundleMethod), "LearnImage");
            }
        } else {
            mBundle.putString(resources.getString(R.string.VABundleMethod), "LearnImage");
        }
    }

    @Override
    public double calculateFamiliarity(int[] image) {

        Log.d(TAG,"calculateFamiliarity Called");

        mBundle.putString(resources.getString(R.string.VABundleMethod), "ErrorCalculation");

        return 0;
    }


    public void getFamiliarity(int[] image){
        Log.d(TAG,"getFamiliarity Called");
        double familiarity = calculateFamiliarity(image);
        Log.d(TAG,"Familiarity = " + familiarity);
        mBundle.putDouble("Error", familiarity);
    }

}
