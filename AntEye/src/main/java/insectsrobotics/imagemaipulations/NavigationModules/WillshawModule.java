package insectsrobotics.imagemaipulations.NavigationModules;

import android.util.Log;

import java.util.Arrays;
import java.util.Random;

import insectsrobotics.imagemaipulations.NavigationModules._Superclasses.NavigationModules;
import insectsrobotics.imagemaipulations.LogToFileUtils;

public class WillshawModule extends NavigationModules {

    int[][] willshawConnectionArray;
    byte[] willshawNetwork;
    int learnedKCs = 0;
    boolean networkSetUp = false;
    boolean firstImage = true;
    boolean inverse = true;
    int threshold = 1750;
    private final String TAG = this.getClass().getSimpleName();
    int maximum_KCs = 0;

    public WillshawModule() {
        super();
        willshawConnectionArray = new int[20000][10];
        Log.e(TAG, "Constructor Called");
        LogToFileUtils.write("Willshaw Module Constructor Called");
    }

    @Override
    public void setupLearningAlgorithm(int[] image) {
        Log.e(TAG, "setupLearningAlgorithm called");
        LogToFileUtils.write("setupLearningAlgorithm called");
        super.setupLearningAlgorithm(image);
        willshawNetwork = new byte[willshawConnectionArray.length];
        if (inverse){
            for (int n = 0; n < image.length; n++){
                image[n] = 255 - image[n];
            }
        }
        Arrays.fill(willshawNetwork, (byte) 1);
        Random random;
        for (int KCNumber = 0; KCNumber < willshawConnectionArray.length; KCNumber++) {
            for (int PNNumber = 0; PNNumber < 10; PNNumber++) {
                random = new Random();
                willshawConnectionArray[KCNumber][PNNumber] = random.nextInt(image.length);
                //Log.e("WillshawModule", "PixelNumber = " + willshawConnectionArray[KCNumber][PNNumber]);
            }
        }
        networkSetUp = true;
    }

    boolean lowered = false;
    boolean raised = false;
    int adjustment = 10;

    @Override
    public void learnImage(int[] image) {
        super.learnImage(image);

        if (firstImage) {
            while (learnedKCs >= 400 || learnedKCs <= 200) {
                Arrays.fill(willshawNetwork, (byte) 1);
                learnedKCs = 0;
                for (int KCNumber = 0; KCNumber < willshawConnectionArray.length; KCNumber++) {
                    int brightness = 0;
                    for (int PNNumber = 0; PNNumber < 10; PNNumber++) {
                        int brightness_tmp = image[willshawConnectionArray[KCNumber][PNNumber]];

                        brightness = brightness + brightness_tmp;
                    }
                    if (brightness >= threshold) {
                        if (willshawNetwork[KCNumber] != 0) {
                            willshawNetwork[KCNumber] = 0;
                            learnedKCs++;
                        }
                    }
                }
                if (learnedKCs >= 400) {
                    if (lowered) {
                        adjustment--;
                    }
                    threshold = threshold + adjustment;
                    raised = true;
                    lowered = false;
                } else if (learnedKCs <= 200) {
                    if (raised) {
                        adjustment--;
                    }
                    threshold = threshold - adjustment;
                    lowered = true;
                    raised = false;
                }
                LogToFileUtils.write("Threshold after adjustment = " + threshold);
                Log.e(TAG, "Threshold after adjustment = " + threshold);
            }
            firstImage = false;
        } else {
            learnedKCs = 0;
            for (int KCNumber = 0; KCNumber < willshawConnectionArray.length; KCNumber++) {
                int brightness = 0;
                for (int PNNumber = 0; PNNumber < 10; PNNumber++) {
                    int brightness_tmp = image[willshawConnectionArray[KCNumber][PNNumber]];
                    brightness = brightness + brightness_tmp;
                }
                if (brightness >= threshold) {
                    if (willshawNetwork[KCNumber] != 0) {
                        willshawNetwork[KCNumber] = 0;
                        learnedKCs++;
                    }
                }
            }
        }
        LogToFileUtils.write("Deactivated Kenyon Cells: " + learnedKCs);
        if (learnedKCs >= maximum_KCs){
            maximum_KCs = learnedKCs;
        }
        //Log.e(TAG, "Deactivated Kenyon Cells: " + learnedKCs);
        learnedKCs = 0;
    }

    @Override
    public double calculateFamiliarity(int[] image) {
        super.calculateFamiliarity(image);
        int error = 0;
        int sum = 0;
        if (networkSetUp) {
            for (int i : willshawNetwork) {
                sum += i;
            }
            Log.e(TAG, "Array Sum: " + sum);
            Log.e(TAG, "Threshold: " + threshold);
            for (int KCNumber = 0; KCNumber < willshawConnectionArray.length; KCNumber++) {
                int brightness = 0;
                for (int PNNumber = 0; PNNumber < 10; PNNumber++) {
                    int brightness_tmp = image[willshawConnectionArray[KCNumber][PNNumber]];
                    brightness = brightness + brightness_tmp;
                }
                if (brightness >= threshold) {
                    error = error + willshawNetwork[KCNumber];
                }
            }
            LogToFileUtils.write("Calculated Error: " + error);
            Log.e(TAG, "Error: " + error);
        }
        return error;
    }

    public double calculateMaximumUnfamiliarity() {
        return maximum_KCs;
    }

}
