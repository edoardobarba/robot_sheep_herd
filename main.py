from model import *

if __name__ == '__main__':
    n_steps = 600
    model = HerdModel(n_robots=4, n_steps = n_steps)

    for i in range(n_steps):
        model.step(i)
        if i%100 == 0:
            print("step: ", i)
    plt.show()  # Show the final plot