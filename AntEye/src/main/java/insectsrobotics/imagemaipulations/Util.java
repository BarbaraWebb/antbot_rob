package insectsrobotics.imagemaipulations;

import java.io.File;
import java.io.FileOutputStream;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.ejml.simple.SimpleMatrix;

import android.os.Environment;
import android.graphics.Bitmap;
import android.util.Log;
import android.os.SystemClock;

//
// Utilities class to hold useful methods
//
public class Util {
    //
    // Singleton construction
    //
    private static Util instance = new Util();
    private Util(){}

    public static Util getInstance()
    {
        return instance;
    }

    //
    // Methods
    //

    //
    // Rotate image matrix and return byte array, used for visual scanning
    //
    public static byte[] rotateMatInAzimuth( int pixels, Mat image_in ) {
        String tag = "OFTST";
        String output = "";

        //IMPORTANT: need to copy the image so we don't modify the original, they're referenced
        //unlike everything else in Java.
        Mat image = new Mat(image_in.rows(), image_in.cols(), image_in.type());
        image_in.copyTo(image);

        byte[] byte_image_array = new byte[image.rows() * image.cols()];

        //Rotate matrix using a temporary matrix to store the rotated columns
        if ( pixels > 0 ) {

            for ( int j = 0; j < pixels; ++j ) {
                Mat first = new Mat(image.rows(), 1, CvType.CV_8UC1);
                Mat tmp = new Mat(image.rows(), 1, CvType.CV_8UC1);
                image.col(0).copyTo(first);
                for ( int i = 1; i < image.cols(); ++i ) {
                    image.col(i).copyTo(image.col(i - 1));
                }
                first.copyTo( image.col(image.cols() - 1) );
            }


        } else if ( pixels < 0 ){
            //Negative pixel reading, right rotation

            for ( int j = 0; j < Math.abs(pixels); ++j ) {
                Mat last = new Mat(image.rows(), 1, CvType.CV_8UC1);
                Mat tmp = new Mat(image.rows(), 1, CvType.CV_8UC1);
                image.col( image.cols() - 1 ).copyTo(last);
                for ( int i = image.cols() - 2; i >= 0; --i ) {
                    image.col(i).copyTo(image.col(i + 1));
                }
                last.copyTo( image.col(0) );
            }

        }

        image.get(0, 0, byte_image_array);  //get byte array from rotated image

        return byte_image_array;
    }


    //
    // Get minimum index of an array
    //
    public static int getMinIndex(double[] arr)
    {
        double minNum = arr[0];
        int flag = 0;
        for(int i = 0; i < arr.length; i++){
            if(arr[i] < minNum){
                minNum = arr[i];
                flag = i;
            }
        }
        return flag;
    }

    //
    // Modulo function
    //
    public static int mod(int x, int y)
    {
        int result = x % y;
        return result < 0 ? result + y : result;
    }

    //
    // Method to save the frames on the DCIM folder in the android phone.
    //
    public static void saveImageToFolder(Mat image, String filename, String tag)
    {
        Bitmap bmp = null;
        tag = "SAVE";

        try
        {
            bmp = Bitmap.createBitmap(image.cols(), image.rows(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(image, bmp);
        }
        catch (Exception e)
        {
            Log.d(tag, e.getMessage());
        }

        FileOutputStream out = null;
        filename += ".png";

        File sd = new File(Environment.getExternalStorageDirectory(),"/DCIM/");
        boolean success = true;
        if (!sd.exists()) {
            success = sd.mkdir();
        }
        if (success) {
            File dest = new File(sd, filename);
            Log.d(tag, dest.getAbsolutePath());
            try {
                out = new FileOutputStream(dest);
                bmp.compress(Bitmap.CompressFormat.PNG, 100, out); // bmp is your Bitmap instance
                // PNG is a lossless format, the compression factor (100) is ignored

            } catch (Exception e) {
                e.printStackTrace();
                Log.d(tag, e.getMessage());
            } finally {
                try {
                    if (out != null) {
                        out.close();
                        Log.d(tag, "OK!!");
                    }
                } catch (Exception e) {
                    Log.d(tag, e.getMessage() + "Error");
                    e.printStackTrace();
                }
            }
        }
    }

    //
    // Deprecated. Function for writing to files.
    //
    public static int log(File file, String output)
    {
        //Function to log information to an output file;
        //Note to whoever inherits this monstrous code: This function does work but its deprecated
        //You should instead use LogToFileUtils; StatFileUtils (see guide or contact me for formatting
        //information); or create your own logging utility based on LogToFileUtils.
        output = output.concat("\n"); //Add an implicit newline character
        try {
            FileOutputStream stream = new FileOutputStream(file);
            stream.write(output.getBytes()); //Write to the output stream
            stream.close();
        } catch (Exception e) { //Catch FileIO exception
            e.printStackTrace();
            System.exit(-1);
            return 1; //Return flag
        }
        return 0;
    }

    //
    // Method to print the contents of the CX network at a given point
    //
    public static String printMemory(SimpleMatrix memory)
    {
        return  "                "+memory.get(0) +"\n"+
                "                "+memory.get(8)+"\n"+
                "      "+memory.get(1)+"     "+memory.get(7)+"\n"+
                "      "+memory.get(9)+"     "+memory.get(15)+"\n"+
                memory.get(2)+"             "+memory.get(6)+"\n"+
                memory.get(10)+"             "+memory.get(14)+"\n"+
                "      "+memory.get(3)+"     "+memory.get(5)+"\n"+
                "      "+memory.get(11)+"     "+memory.get(13)+"\n"+
                "                "+memory.get(4)+"\n"+
                "                "+memory.get(12);
    }

    //
    // Calculate the rms difference between two images
    //
    private double rmsDifference(Mat current, Mat ref)
    {
        int counter1 = 0;
        int counter3;
        double rootSum=0;
        int count =0;
        for (int cols = 0; cols < current.cols(); cols++) {
            counter3 = 0;
            for (int rows = 0; rows < current.rows(); rows++) {
                double xDiff = current.get(counter3,counter1)[0]-ref.get(counter3,counter1)[0];
                rootSum = rootSum + (Math.pow(xDiff,2));
                count++;
                counter3++;

            }
            counter1++;
        }
        return (Math.sqrt(rootSum/count));
    }

    //
    // Rotate a 90x10 image (4 degrees intervals)
    //
    public static Mat rotateImage(Mat image, int theta)
    {
        int counter1 = 0;
        int counter3;
        int colPosition;
        double[] rotPixel;
        if(theta<0){
            theta=theta+360;
        }
        theta = theta/4;
        Mat rotatedImage = Mat.zeros(image.size(),image.type());

        for (int phi = 0; phi < image.cols(); phi++) {
            counter3 = 0;
            for (int theta_tmp = 0; theta_tmp < image.rows(); theta_tmp++) {
                colPosition=counter1+theta;
                if(colPosition >=90){
                    colPosition=colPosition - 90;
                }
                rotPixel = image.get(counter3,colPosition);
                rotatedImage.put(counter3, counter1, rotPixel);
                counter3++;
            }
            counter1++;
        }

        return rotatedImage;
    }


    //
    // Function to write the CX network out to file
    //
    public static void writeToFile(
            SimpleMatrix memory,
            double left_speed,
            double right_speed,
            String filename,
            float frame_rate,
            int iteration,
            double heading
    )
    {

        File sd = new File(Environment.getExternalStorageDirectory(),"/DCIM/");
        filename += ".txt";
        File dest = new File(sd, filename);
        try {
            FileOutputStream stream = new FileOutputStream(dest, true);
            String logging_info = get_logging_data(
                    memory,
                    left_speed,
                    right_speed,
                    frame_rate,
                    iteration,
                    heading);
            stream.write(logging_info.getBytes());
            stream.flush();
            stream.close();
        } catch (Exception e){
            e.printStackTrace();
        }
    }

    private static String get_logging_data(
            SimpleMatrix memory,
            double left_speed,
            double right_speed,
            float frame_rate,
            int iteration,
            double heading
    )
    {
        int time = (int) SystemClock.elapsedRealtime();
        return  ""+myFormat(frame_rate)+" || "+myFormat(iteration)+" || "+
                ""+myFormat(heading)+" || "+myFormat(time/1000)+" || "+
                ""+myFormat(left_speed)+"|"+ myFormat(right_speed)+" || "+
                ""+myFormat(memory.get(0))+","+myFormat(memory.get(8))+"|" +
                ""+myFormat(memory.get(7))+","+myFormat(memory.get(15))+"|" +
                ""+myFormat(memory.get(6))+","+myFormat(memory.get(14))+"|" +
                ""+myFormat(memory.get(5))+","+myFormat(memory.get(13))+"|" +
                ""+myFormat(memory.get(4))+","+myFormat(memory.get(12))+"|" +
                ""+myFormat(memory.get(3))+","+myFormat(memory.get(11))+"|" +
                ""+myFormat(memory.get(2))+","+myFormat(memory.get(10))+"|" +
                ""+myFormat(memory.get(1))+","+myFormat(memory.get(9))+"\n";
    }

    private static String myFormat(float num){
        if (num < 0) return String.format("%02.4f", num);
        else return " "+String.format("%02.4f", num);
    }

    private static String myFormat(double num){
        if (num < 0) return String.format("%02.4f", num);
        else return " "+String.format("%02.4f", num);
    }

    private static String myFormat(int num){
        if (num < 0) return String.format("%5d", num);
        else return " "+String.format("%5d", num);
    }

    public static boolean isHome(SimpleMatrix memory){
        int time = (int) SystemClock.elapsedRealtime();
        int count = 0;

        for(int i = 0; i < 16; i++){
            if (Math.abs(memory.get(i) - 0.5) < 0.01){
                count ++;
            }
        }

        return count>14 & time>45000;
    }

    //
    // Given two flow frames and a 2x1 Mat foe, compute the focus of expansion from prevPoints and
    // currentPoints; store the result in the foe Mat. I don't trust returns using Matrices. The
    // computation is done using the LSE method from Tistrelli et al.
    //
    public static void computeFocusOfExpansion (Mat prevPoints, Mat currentPoints, Mat foe){
        String tag = "FOEC";
        Mat A = Mat.ones(900, 2, CvType.CV_32FC1);
        Mat b = Mat.ones(900, 1, CvType.CV_32FC1);

        int k = 0;
        for (int i = 0; i < currentPoints.rows(); i = i + 2){
            for (int j = 23; j < currentPoints.cols() - 23; j = j + 5){
                //
                // Extract flow vector f = (u,v) from the vector
                //
                double[] flow = currentPoints.get(i,j);

                // bk = xv - yu --> bk = i(flow[1]) - j(flow[0]) (from O'Donovan Optic Flow paper).
                // u = flow[0]
                // v = flow[1]
                // x = j
                // y = i
                int bk = (j * (int) flow[1]) - (i * (int) flow[0]);

                // Place the correct data into the kth rows of b and A.
                b.put(k, 0, bk);
                A.put(k, 0, flow[1]);
                A.put(k, 1, flow[0]);

                k++;
            }
        }

        //
        // Now have complete A and b construction; now we compute the FOE as:
        // FOE = (A'A)^-1A'b; where A' is the matrix transpose
        //

        // A_inv = (A'A)^-1
        Mat A_inv = new Mat();
        Log.i(tag, "A size: " + A.rows() + "x" + A.cols());
        Core.gemm(A.t(), A, 1, new Mat(), 0, A_inv);
        //Log.i(tag, "A_inv size: " + A_inv.rows() + "x" + A_inv.cols());
        A_inv = A_inv.inv();


        // A = ((A'A)^-1)A'
        Core.gemm(A_inv, A.t(), 1, new Mat(), 0, A);
        //Log.i(tag, "A size where A = ((A'A)^-1)A': " + A.rows() + "x" + A.cols());

        // foe = ((A'A)^-1)A'b
        // Again we transpose the first matrix passed in to "undo" the auto-transpose
        // Laws of matrix multiplication state that this should be a 2x1 (as foe)
        Core.gemm(A, b, 1, new Mat(), 0, foe);

        Log.i(tag, "FOE : " + foe.dump());

        // Number formatting.
        int amp = 10; // Amplification factor
        foe.put(0,0, Math.abs(foe.get(0,0)[0])*10 % 90);
        foe.put(1,0, Math.abs(foe.get(1,0)[0])*10 % 90);
        //foe.put(0,0, 45);
        //foe.put(1,0,5);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////

    //
    // Unused code that may still prove useful. This code is not ready for use but may provide help.
    // Convention is to comment the code within the function. This allows us to use
    // Android Studio's folding feature to hide the code.
    //

    //
    // Image rotation and down sampling. Could split this into two functions.
    // Refers to old globals and some libs not yet imported here.
    //
    public static Mat rotateFullImage(Mat pic, int angle){
        /*
        Commented out because the function structure will not work as intended
        Use rotateMatInAximuth with a pixel rotation


        Mat processedMat;
        processedMat=Mat.zeros(processedDestImage.size(),processedDestImage.type());
        int counter1;
        int counter3=0;
        int colPosition;
        Mat tempRotate;
        double[] pixelNew;

        tempRotate=new Mat(fullSnapShot.size(),fullSnapShot.type());
        for (int theta_tmp = 0; theta_tmp < 40; theta_tmp = theta_tmp + 1) {
            counter1 = 0;
            for (int phi = 0; phi < 360; phi = phi + 1) {
                colPosition=counter1+angle;
                if(colPosition>=360){
                    colPosition=colPosition - 360;
                }
                pixelNew = pic.get(counter3,colPosition);
                tempRotate.put(counter3, counter1, pixelNew);

                counter1++;
            }
            counter3++;
        }

        //downsampling
        int destAzimuthCounter = 0;
        for (int azimuth = 0; azimuth < tempRotate.cols(); azimuth = azimuth + 4) {
            int destElevationCounter = 0;
            for (int elevation = 0; elevation < tempRotate.rows(); elevation = elevation + 4) {
                Rect roi;
                //New Rectangle with the target resolution, later the ROI of the frame.
                roi = new Rect(azimuth, elevation, 4, 4);
                if (tempRotate.cols() - azimuth < 4) {
                    roi = new Rect(azimuth, elevation, tempRotate.cols() - azimuth, 4);
                }
                if (tempRotate.rows() - elevation < 4) {
                    roi = new Rect(azimuth, elevation, 4, tempRotate.rows() - elevation);
                }
                //Getting the pixels of the Region of Interest and averaging the values.
                Mat ROI = tempRotate.submat(roi);
                int ROIMean = (int) Core.mean(ROI).val[0];
                processedMat.put(destElevationCounter, destAzimuthCounter, ROIMean);
                destElevationCounter++;

            }
            destAzimuthCounter++;
        }

        Imgproc.equalizeHist(processedMat, processedMat);

        GaussianBlur(processedMat, processedMat, new Size(3, 3), 0, 0);
        broadcast.broadcastRotatedImage(processedMat);
        return processedMat;
        */
        return new Mat();
    }

    //
    // Image rotation without down sampling. Refers to old globals.
    //
    private static void rotateSnapShot(){
        /*
        int counter1 = 0;
        int counter3;
        int colPosition;
        Mat tempRotate;
        tempRotate=Mat.zeros(fullSnapShot.size(),fullSnapShot.type());
        for (int phi = 0; phi < 360; phi = phi + 1) {
            counter3 = 0;
            for (int theta_tmp = 0; theta_tmp < 40; theta_tmp = theta_tmp + 1) {
                colPosition=counter1+180;
                if(colPosition>=360){
                    colPosition=colPosition - 360;
                }
                double[] pixelNew = fullSnapShot.get(counter3,colPosition);
                tempRotate.put(counter3, counter1, pixelNew);
                counter3++;
            }
            counter1++;
        }
        tempRotate.copyTo(fullSnapShot);
*/
    }
}
