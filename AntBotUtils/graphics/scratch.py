from trajectory import Trajectory
import matplotlib.pyplot as plt

if __name__ == '__main__':
    path = "/home/robert/Uni/project/antbot_rob/notes/2/data/cx_csv/"
    out_path = path + "CX_out_test.csv"
    in_path = path + "CX_in_test.csv"
    traj_plot = Trajectory(out_path, in_path, clean=True)

    plt.ylim((1100, -1100))
    plt.xlim((-1500, 2500))
    plt.xlabel('Distance (mm)')
    plt.ylabel('Distance (mm)')
    plt.title("Central Complex No-turn Test (Orange - Outbound; Blue - Inbound)")
    plt.grid(True)
    plt.show()
