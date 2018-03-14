package insectsrobotics.imagemaipulations.NavigationModules;

import android.util.Log;

import java.util.Arrays;
import java.util.Random;

import insectsrobotics.imagemaipulations.NavigationModules._Superclasses.NavigationModules;
import insectsrobotics.imagemaipulations.LogToFileUtils;
import insectsrobotics.imagemaipulations.StatFileUtils;

public class WillshawModule extends NavigationModules {

    int[][] willshawConnectionArray;
    byte[] willshawNetwork;
    int learnedKCs = 0;
    boolean networkSetUp = false;
    boolean firstImage = true;
    boolean inverse = true;
    int threshold = 1700;
    float f_threshold = 5.25f;
    int f_learnedKCs = 0;
    private final String TAG = this.getClass().getSimpleName();
    int maximum_KCs = 0;

    public WillshawModule() { //Willshaw constructor
        super(); //Call super
        willshawConnectionArray = new int[20000][10]; // Connection array, 20,000 KCs each with input from 10 random vPNs
        Log.e(TAG, "Constructor Called");
        LogToFileUtils.write("Willshaw Module Constructor Called");
    }

    @Override
    public void setupLearningAlgorithm(int[] image) {
        Log.e(TAG, "setupLearningAlgorithm called");
        LogToFileUtils.write("setupLearningAlgorithm called");
        super.setupLearningAlgorithm(image);
        willshawNetwork = new byte[willshawConnectionArray.length]; //Willshaw network of size 20,000KCs
        //image = normalisePNs(image);

        if (inverse){
            for (int n = 0; n < image.length; n++){
                image[n] = 255 - image[n];
            }
        }
        Arrays.fill(willshawNetwork, (byte) 1);
        Random random;

        //byte []connected = new byte[image.length];
        for (int KCNumber = 0; KCNumber < willshawConnectionArray.length; KCNumber++) { //For each KC
            for (int PNNumber = 0; PNNumber < 10; PNNumber++) { //For each input vPN for that KC
                random = new Random();
                willshawConnectionArray[KCNumber][PNNumber] = random.nextInt(image.length); //Tie that KC to a random pixel
                //Log.e("WillshawModule", "PixelNumber = " + willshawConnectionArray[KCNumber][PNNumber]);
                //connected[willshawConnectionArray[KCNumber][PNNumber]] = 1;

            }
        }
        //StatFileUtils.write("WN", "CON", Arrays.toString(connected));
        networkSetUp = true;
    }

    boolean lowered = false;
    boolean raised = false;
    int adjustment = 10;

    @Override
    public void learnImage(int[] image) {
        super.learnImage(image);
        float[] float_image = new float[image.length];
        int total_KCs = 0;
        //Log.e("WNNorm", "Pixel values before normalisation: " + Arrays.toString(float_image));
        float_image = normalisePNs(image); //Normalise the image
        //Log.e("WNNorm", "Pixel values after normalisation: " + Arrays.toString(float_image));
        if (firstImage) {
            while (learnedKCs >= 400 || learnedKCs <= 200) {
                Arrays.fill(willshawNetwork, (byte) 1); //Reset the network for this iteration
                learnedKCs = 0;
                total_KCs = 0;
                f_learnedKCs = 0;
                for (int KCNumber = 0; KCNumber < willshawConnectionArray.length; KCNumber++) { // For each KC
                    int brightness = 0; // Coincident brightness
                    float f_brightness = 0;
                    for (int PNNumber = 0; PNNumber < 10; PNNumber++) { // For each associated projected neuron
                        //Get the pixel brightness for the pixel related to the KC, PN connection
                        int brightness_tmp = image[willshawConnectionArray[KCNumber][PNNumber]]; //Pixel brightness
                        float brightness_flt = float_image[willshawConnectionArray[KCNumber][PNNumber]];
                        brightness = brightness + brightness_tmp; //Add to the total brightness
                        f_brightness = f_brightness + brightness_flt;
                    }
                    //Log.e("WNN", "Brightness (float): " + f_brightness);
                    int wn_copy = willshawNetwork[KCNumber];
                    if (brightness >= threshold) { //If coincident brightness exceeds threshold
                        if (willshawNetwork[KCNumber] != 0) { //If KC has not yet been used for learning
                            willshawNetwork[KCNumber] = 0; //Lower weight to zero
                            learnedKCs++; //Increment number of KCs used for learning this image
                            total_KCs++;
                        }
                    }

                    if ( f_brightness >= f_threshold ){ if ( wn_copy != 0 ){f_learnedKCs++;} }
                }
                if (learnedKCs >= 400) { //If at least 400KCs have been used for learning
                    if (lowered) {
                        adjustment--; //Reduce adjustment by 1
                    }
                    threshold = threshold + adjustment; // Adjust threshold accordingly

                    f_threshold = f_threshold + ((float) adjustment / 100);

                    raised = true; //Adjustment was just raised
                    lowered = false;
                } else if (learnedKCs <= 200) { //If used less than 200KCs for learning
                    if (raised) { //If we just raised the threshold
                        adjustment--; //Reduce adjustment by 1
                    }
                    threshold = threshold - adjustment; //Reduce threshold

                    f_threshold = f_threshold - ((float) adjustment / 100);

                    lowered = true; //Threshold was just lowered
                    raised = false;
                }
                LogToFileUtils.write("Threshold after adjustment = " + threshold);
                Log.e("WNN", "Threshold after adjustment = " + threshold);
                Log.e("WNN", "Deactivated KCs for first image " + learnedKCs);
            }
            firstImage = false; //Have now processed the first image.
        } else {
            learnedKCs = 0; //Init learned KCs to zero for the new image

            f_learnedKCs = 0;
            for (int KCNumber = 0; KCNumber < willshawConnectionArray.length; KCNumber++) { //For each KC
                int brightness = 0;
                float f_brightness = 0;
                for (int PNNumber = 0; PNNumber < 10; PNNumber++) { //For each projection neuron
                    //Get brightness for pixel associated with KC, PN connection
                    int brightness_tmp = image[willshawConnectionArray[KCNumber][PNNumber]];
                    float brightness_flt = float_image[willshawConnectionArray[KCNumber][PNNumber]];
                    brightness = brightness + brightness_tmp; //Update brightness
                    f_brightness = f_brightness + brightness_flt;
                }

                if (brightness >= threshold) { //If brightness above threshold
                    total_KCs++;
                    if (willshawNetwork[KCNumber] != 0) { //If KC not already been used
                        willshawNetwork[KCNumber] = 0; //Decrease weight for KC
                        learnedKCs++; //Increment #of kcs used

                        if ( f_brightness >= f_threshold ){ f_learnedKCs++;}
                    }
                }
            }
        }
        LogToFileUtils.write("Deactivated Kenyon Cells: " + learnedKCs);
        if (learnedKCs >= maximum_KCs){ //Update max KCs if necessary
            maximum_KCs = learnedKCs;

        }
        Log.e("WNN", "Deactivated Kenyon Cells: " + learnedKCs);
        Log.e("WNN", "Total Kenyon Cells down: " + total_KCs);
        Log.e("WNN", "Deactivated Kenyon Cells (float): " + f_learnedKCs);
        StatFileUtils.write("WNN", "KCS", "Learning, total kenyon cells: " + total_KCs);
        StatFileUtils.write("WNN", "KCS", "Learning, learned kenyon cells: " + learnedKCs);
        learnedKCs = 0;
    }

    @Override
    public double calculateFamiliarity(int[] image) {
        super.calculateFamiliarity(image);
        //image = normalisePNs(image);
        int error = 0; // Needs to be initialised to non-zero
        int sum = 0;
        int activated = 0;
        if (networkSetUp) {
            for (int i : willshawNetwork) { sum += i; } //number of KCs not used for any learning
            //error = sum; //Max it could possibly be

            Log.e(TAG, "Array Sum: " + sum);
            Log.e(TAG, "Threshold: " + threshold);
            //StatFileUtils.write("WN", "SUM", "Sum: " + sum);

            for (int KCNumber = 0; KCNumber < willshawConnectionArray.length; KCNumber++) {
                int brightness = 0;
                for (int PNNumber = 0; PNNumber < 10; PNNumber++) {
                    int brightness_tmp = image[willshawConnectionArray[KCNumber][PNNumber]];
                    brightness = brightness + brightness_tmp;
                }
                if (brightness >= threshold) {
                    //StatFileUtils.write("WN", "BGT", "Brightness: " + brightness);
                    error = error + willshawNetwork[KCNumber]; //If KC was used for learning, willshaw at [KCn] will be 0.
                    activated++;
                }
            }
            LogToFileUtils.write("Calculated Error: " + error);
            Log.e("WNN", "Error: " + error);
        }


        //If < 100 KCs are activated, or more KCs are activated than were activated by the image
        //which learned the most, then set to the maximum.

        if (activated < 100 || activated > 600){ error = maximum_KCs; }

        StatFileUtils.write("WN", "KCX", "KCs activated for image: " + activated);
        Log.e("WNN", "Activated: " + activated);
        return error;
    }

    public float[] normalisePNs(int[] image) { //Normalise the image by normalising the PNs
        double sum = 0;
        float[] float_image = new float[image.length];
        for ( int i = 0; i < image.length; ++i ){ sum+=image[i]; }
        sum = Math.sqrt(sum);
        for ( int i = 0; i < image.length; ++i ){ float_image[i] = (float) image[i] / (float) sum; }

        return float_image;
    }

    //Zhaoyu?..
    public double calculateMaximumUnfamiliarity() {
        return maximum_KCs;
    }

}
