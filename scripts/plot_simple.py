import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":
    # Generate some data
    x = np.arange(1,10)
    y = np.array([83.3, 58.9, 44.2, 37.0, 29.4, 25.7, 22.7, 20.3, 18.2])
    z = np.array([166.9, 111.3, 83.6, 68.4, 55.6, 48.0, 43.0, 38.5, 34.5])

    # Create a simple plot
    plt.plot(x, y, label='Sine Wave')
    plt.plot(x, z, label='Cosine Wave', linestyle='--')
    plt.title('Simple Plot Example')
    plt.xlabel('X-axis')
    plt.ylim([0, 200])
    plt.ylabel('Y-axis')
    plt.legend()
    
    # Show the plot
    plt.show()