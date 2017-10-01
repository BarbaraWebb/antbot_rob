package insectsrobotics.imagemaipulations.NavigationModules;


import android.content.res.Resources;
import android.util.Log;

import java.util.ArrayList;
import java.util.List;

import insectsrobotics.imagemaipulations.NavigationModules._Superclasses.NavigationModules;

public class PerfectMemoryModule extends NavigationModules {

    private final String TAG = this.getClass().getSimpleName();

    List<int[]> learnedImages;

public PerfectMemoryModule() {
    super();
}

    @Override
    public void setupLearningAlgorithm(int[] image) {
        super.setupLearningAlgorithm(image);
        learnedImages = new ArrayList<>();
    }

    @Override
    public void learnImage(int[] image) {
        super.learnImage(image);
        Log.e(TAG, "PM Image Length: " + image.length);
        learnedImages.add(image);
    }

    @Override
    public double calculateFamiliarity(int[] image) {
        super.calculateFamiliarity(image);
        double error = 0;
        double smallestError = 0;
        Log.e(TAG,"List length: " + learnedImages.size());
        for (int imageCounter = 0; imageCounter < learnedImages.size(); imageCounter++){
            int[] learnedImage = learnedImages.get(imageCounter);
            int pixelCounter;
            for (pixelCounter = 0; pixelCounter < image.length; pixelCounter++){
                int difference = (learnedImage[pixelCounter])-(image[pixelCounter]);
                int squaredError = difference * difference;
                error = error + squaredError;
            }
            error = Math.sqrt(error/pixelCounter);
            if (imageCounter != 0){
                if (error < smallestError){
                    smallestError = error;
                }
            } else {
                smallestError = error;
            }
        }
        return smallestError;
    }


    public double calculateMaximumUnfamiliarity() {
        int[] firstImage = learnedImages.get(0);
        double maximum_unfamiliarity = 0;
        double error = 0;

        for (int imageCounter = 0; imageCounter < learnedImages.size(); imageCounter++){
            int[] learnedImage = learnedImages.get(imageCounter);
            int pixelCounter;
            for (pixelCounter = 0; pixelCounter < firstImage.length; pixelCounter++){
                int difference = (learnedImage[pixelCounter])-(firstImage[pixelCounter]);
                int squaredError = difference * difference;
                error = error + squaredError;
            }
            error = Math.sqrt(error/pixelCounter);

            if (error >= maximum_unfamiliarity){
                maximum_unfamiliarity = error;
            }
        }
        return maximum_unfamiliarity;
    }
}
