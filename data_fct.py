import csv
import matplotlib.pyplot as plt

def plot_line(data, idx_x=0, idx_y=1, dmin=-1, dmax=1, rmin=-1, rmax=1):
    if len(data) == 1:
        plt.plot(data[idx_x])
    else:
        plt.plot(data[idx_x], data[idx_y])

def plot_scatter(data, idx_x=0, idx_y=1, dmin=-1, dmax=1, rmin=-1, rmax=1):
    plt.scatter(data[idx_x], data[idx_y])

def set_up_plot(fig_num, dmin=None, dmax=None, rmin=None, rmax=None):
    figure = plt.figure(fig_num)
    panel = figure.add_subplot(111)

    if (dmin is not None
            and dmax is not None
            and rmin is not None
            and rmax is not None):
        panel.set_ylim(rmin, rmax)
        panel.set_xlim(dmin, dmax)

def show_plot():
    plt.show()

def write_csv(filename, data_names, data):
    file = open(filename, 'w', newline='')
    writer = csv.DictWriter(file, fieldnames=data_names)

    writer.writeheader()

    # iterate over all rows
    for i in range(0, len(data[0])):
        d = {}

        # iterate over all columns
        for j, name in enumerate(data_names):
            if len(data[j]) > i:
                d[name] = data[j][i]

        writer.writerow(d)

    file.close()

