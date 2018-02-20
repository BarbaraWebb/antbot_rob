package insectsrobotics.imagemaipulations.NavigationModules;

import android.util.Log;

import java.util.Arrays;
import java.util.Random;

import insectsrobotics.imagemaipulations.LogToFileUtils;
import insectsrobotics.imagemaipulations.NavigationModules._Superclasses.NavigationModules;

public class RealWillshawModule extends NavigationModules {

    int[][] willshawConnectionArray;
    float[] willshawNetwork;
    int learnedKCs = 0;
    boolean networkSetUp = false;
    boolean firstImage = true;
    boolean inverse = true;
    int threshold = 1750;
    private final String TAG = this.getClass().getSimpleName();
    int maximum_KCs = 0;
    float weight = 1; //Initial weighting for KCs
    float rate = 0.9f; //Learning rate for KCs (on activation weight = weight * rate;)

    public RealWillshawModule() { //Willshaw constructor
        super(); //Call super
        willshawConnectionArray = new int[20000][10]; // Connection array, 20,000 KCs each with input from 10 random vPNs
        Log.e(TAG, "Constructor Called");
        LogToFileUtils.write("Willshaw Module Constructor Called");
    }

    @Override
    public void setupLearningAlgorithm(int[] image) {
        //The image isn't used at all so this could potentially be refactored; the only use is the image inversion
        Log.e(TAG, "setupLearningAlgorithm called");
        LogToFileUtils.write("setupLearningAlgorithm called");
        super.setupLearningAlgorithm(image);
        willshawNetwork = new float[willshawConnectionArray.length]; //Willshaw network of size 20,000KCs
        if (inverse){
            for (int n = 0; n < image.length; n++){
                image[n] = 255 - image[n];
            }
        }
        Arrays.fill(willshawNetwork, (float) 1);
        Random random;
        for (int KCNumber = 0; KCNumber < willshawConnectionArray.length; KCNumber++) { //For each KC
            for (int PNNumber = 0; PNNumber < 10; PNNumber++) { //For each input vPN for that KC
                random = new Random();
                willshawConnectionArray[KCNumber][PNNumber] = random.nextInt(image.length); //Tie that KC to a random pixel
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
                Arrays.fill(willshawNetwork, (float) 1); //Make sure initialised
                learnedKCs = 0;
                for (int KCNumber = 0; KCNumber < willshawConnectionArray.length; KCNumber++) { // For each KC
                    int brightness = 0; // Pixel brightness
                    for (int PNNumber = 0; PNNumber < 10; PNNumber++) { // For each associated projected neuron
                        //Get the pixel brightness for the pixel related to the KC, PN connection
                        int brightness_tmp = image[willshawConnectionArray[KCNumber][PNNumber]];

                        brightness = brightness + brightness_tmp; //Update the brightness
                    }

                    if (brightness >= threshold) { //If brightness exceeds threshold
                        if (willshawNetwork[KCNumber] == 1) { //If KC has not yet been used for learning
                            learnedKCs++; //Increment number of KCs used for learning (may be ok to use for real)
                        }
                        willshawNetwork[KCNumber] = willshawNetwork[KCNumber] * rate; //Mult by learning rate
                    }
                }
                if (learnedKCs >= 400) { //If at least 400KCs have been used for learning
                    if (lowered) {
                        adjustment--; //Reduce adjustment by 1
                    }
                    threshold = threshold + adjustment; // Adjust threshold accordingly
                    raised = true; //Threshold was just raised
                    lowered = false;
                } else if (learnedKCs <= 200) { //If used less than 200KCs for learning
                    if (raised) { //If we just raised the threshold
                        adjustment--; //Reduce adjustment by 1
                    }
                    threshold = threshold - adjustment; //Reduce threshold
                    lowered = true; //Adjustment was just lowered
                    raised = false;
                }
                LogToFileUtils.write("Threshold after adjustment = " + threshold);
                Log.e(TAG, "Threshold after adjustment = " + threshold);
            }
            firstImage = false; //Have now processed the first image.
        } else {
            learnedKCs = 0; //Init learned KCs to zero for the new image
            for (int KCNumber = 0; KCNumber < willshawConnectionArray.length; KCNumber++) { //For each KC
                int brightness = 0;
                for (int PNNumber = 0; PNNumber < 10; PNNumber++) { //For each projection neuron
                    //Get brightness for pixel associated with KC, PN connection
                    int brightness_tmp = image[willshawConnectionArray[KCNumber][PNNumber]];
                    brightness = brightness + brightness_tmp; //Update brightness
                }
                if (brightness >= threshold) { //If brightness above threshold
                    if (willshawNetwork[KCNumber] == 1) { //If KC not already been used
                        learnedKCs++; //Increment #of kcs used
                    }

                    willshawNetwork[KCNumber] = willshawNetwork[KCNumber] * rate; //Decrease weight for KC
                }
            }
        }
        LogToFileUtils.write("Deactivated Kenyon Cells: " + learnedKCs);
        if (learnedKCs >= maximum_KCs){ //Update max KCs if neccessary
            maximum_KCs = learnedKCs;
        }
        //Log.e(TAG, "Deactivated Kenyon Cells: " + learnedKCs);
        learnedKCs = 0;
    }

    @Override
    public double calculateFamiliarity(int[] image) {
        super.calculateFamiliarity(image);
        float error = 0;
        int sum = 0;
        if (networkSetUp) {
            for (float i : willshawNetwork) {
                sum += i; //This is used for?...
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
                    error = error + willshawNetwork[KCNumber]; //If KC was used for learning, willshaw at [KCn] will be 0.
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
