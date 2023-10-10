from model import *

if __name__ == '__main__':
    n_steps = 500
    model = HerdModel(n_robots=4, n_steps = n_steps, fixed_target=True)

    for i in range(n_steps):
        #print("step: ", i)

        if i%50 == 0:
            print("step: ", i)
        model.step(i)
    plt.show()  # Show the final plot