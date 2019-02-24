#
# Imports
#
import pandas as pd
import os
import shutil
import matplotlib.pyplot as plt


#
# Class to represent AntBot trajectory plots recorded from Vicon.
#
class Trajectory:
    #
    # Remove the unnecessary information from the dataframe
    #
    def clean_file(self, csv_path):
        print("Backup original file...")
        backup = csv_path + ".backup"
        shutil.copy(csv_path, backup)

        try:
            with open(csv_path, "r") as f:
                print("Opening file for cleaning:")
                print(csv_path)
                data = f.readlines()
                data = [x for x in data if x != "Objects\r\n"]
                data = [x for x in data if x != "100\r\n"]

                # Break into lists for easy editing
                data = [x.split(",") for x in data]

                # Remove any line which starts with an empty character
                data = [x for x in data if x[0] != '']


                # Trim the column header line to the correct length if required

                #if len(data[0]) > 8:
                #    data[0] = data[0][0:8]

                # Clean untracked object data or formatting characters from the file
                for idx in range(len(data)):
                    if len(data[idx]) > 8:
                        data[idx] = data[idx][0:8]
                    data[idx] = [x for x in data[idx] if x != '\r\n']
                    data[idx] = [x for x in data[idx] if x != '\n']

                data = [x for x in data if x != []]

                # Rejoin strings with comma separators
                data = [",".join(x) for x in data]

            # Remove the existing file
            os.remove(csv_path)

            with open(csv_path, "w") as f:

                print("Writing file...")
                for line in data:
                    f.write(line + "\n");

            print("Cleaned data written to:")
            print(csv_path)

        except IOError:
            print("IOError encountered!")
            print("Restoring backup...")
            shutil.copy(backup, csv_path)
            print("Exiting")
            exit(-1)

        except:
            # Not great practice, but if an error occurs then the file appears
            # to get wiped which is unacceptable.
            print("Unexpected error encountered!")
            print("Restoring backup file...")
            shutil.copy(backup, csv_path)
            print("Exiting")
            exit(-1)

        return csv_path

    def clean_frame(self, dataframe, drop_frames=True, drop_rotation=True):
        # Drop the Z axis as we do not care about height
        dataframe.drop("TZ", axis=1, inplace=True)

        #
        # If we want to drop frame counts, drop them here, assumed True
        #
        if drop_frames:
            dataframe.drop("Frame", axis=1, inplace=True)
            dataframe.drop("Sub Frame", axis=1, inplace=True)

        #
        # If we want to drop rotation, drop it here, again, assumed True
        #
        if drop_rotation:
            dataframe.drop("RX", axis=1, inplace=True)
            dataframe.drop("RY", axis=1, inplace=True)
            dataframe.drop("RZ", axis=1, inplace=True)

        # Return cleaned frame
        return dataframe

    def generate_plot(self, clean_frame_list):
        ax1 = clean_frame_list[0].plot(x="TY", y="TX", legend=False)
        for idx in range(1,len(clean_frame_list)):
            clean_frame_list[idx].plot(x="TY", y="TX", legend=False, ax=ax1)

        return ax1

    #
    # Ctor; expects the filename of the inbound/outbound trajectory csvs
    #
    def __init__(self, outbound_plot_filename, inbound_plot_filename, clean=False):
        # Clean data if desired; cleaning a clean csv file will not damage the
        # file.
        if clean:
            self.clean_file(outbound_plot_filename)
            self.clean_file(inbound_plot_filename)

        # Create dataframes from the cleaned csv data
        self.in_trajectory_df = pd.DataFrame()
        self.out_trajectory_df = pd.DataFrame()
        self.in_trajectory_df = pd.read_csv(inbound_plot_filename, error_bad_lines=False)
        self.out_trajectory_df = pd.read_csv(outbound_plot_filename, error_bad_lines=False)

        self.in_trajectory_df = self.clean_frame(self.in_trajectory_df)
        self.out_trajectory_df = self.clean_frame(self.out_trajectory_df)

        frame_list = [self.in_trajectory_df, self.out_trajectory_df]

        self.axes = self.generate_plot(frame_list)
